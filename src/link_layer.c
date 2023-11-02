// Link layer protocol implementation

#include "link_layer.h"

volatile int STOP = FALSE;
int alarmSignaled = FALSE;
int alarmCount = 0;
int timeout = 0;
int retransmitions = 0;
unsigned char tramaTx = 0;
unsigned char tramaRx = 1;

int llopen(LinkLayer connectionParams) {
    LinkLayerState currentState = START;
    int fileDescriptor = connection(connectionParams.serialPort);

    if (fileDescriptor < 0) return -1;

    unsigned char receivedByte;
    timeout = connectionParams.timeout;
    retransmitions = connectionParams.nRetransmissions;

    if (connectionParams.role == LlTx) {
        signal(SIGALRM, alarmHandler);
        while (connectionParams.nRetransmissions != 0 && currentState != STOP_R) {
            sendSupervisionFrame(fileDescriptor, A_ER, C_SET);
            alarm(connectionParams.timeout);
            alarmSignaled = FALSE;

            while (!alarmSignaled && currentState != STOP_R) {
                if (read(fileDescriptor, &receivedByte, 1) > 0) {
                    switch (currentState) {
                        case START:
                            currentState = (receivedByte == FLAG) ? FLAG_RCV : START;
                            break;
                        case FLAG_RCV:
                            currentState = (receivedByte == A_RE) ? ADDRESS_RCV :
                                           (receivedByte == FLAG) ? FLAG_RCV : START;
                            break;
                        case ADDRESS_RCV:
                            currentState = (receivedByte == C_UA) ? CONTROL_RCV :
                                           (receivedByte == FLAG) ? FLAG_RCV : START;
                            break;
                        case CONTROL_RCV:
                            currentState = (receivedByte == (A_RE ^ C_UA)) ? BCC1_OK :
                                           (receivedByte == FLAG) ? FLAG_RCV : START;
                            break;
                        case BCC1_OK:
                            currentState = (receivedByte == FLAG) ? STOP_R : START;
                            break;
                        default:
                            break;
                    }
                }
            }
            connectionParams.nRetransmissions--;
        }
        if (currentState != STOP_R) return -1;
    } else if (connectionParams.role == LlRx) {
        while (currentState != STOP_R) {
            if (read(fileDescriptor, &receivedByte, 1) > 0) {
                switch (currentState) {
                    case START:
                        currentState = (receivedByte == FLAG) ? FLAG_RCV : START;
                        break;
                    case FLAG_RCV:
                        currentState = (receivedByte == A_ER) ? ADDRESS_RCV :
                                       (receivedByte == FLAG) ? FLAG_RCV : START;
                        break;
                    case ADDRESS_RCV:
                        currentState = (receivedByte == C_SET) ? CONTROL_RCV :
                                       (receivedByte == FLAG) ? FLAG_RCV : START;
                        break;
                    case CONTROL_RCV:
                        currentState = (receivedByte == (A_ER ^ C_SET)) ? BCC1_OK :
                                       (receivedByte == FLAG) ? FLAG_RCV : START;
                        break;
                    case BCC1_OK:
                        currentState = (receivedByte == FLAG) ? STOP_R : START;
                        break;
                    default:
                        break;
                }
            }
        }
        sendSupervisionFrame(fileDescriptor, A_RE, C_UA);
    } else {
        return -1;
    }

    return fileDescriptor;
}

int connection(const char* port) {
    int serialFd = open(port, O_RDWR | O_NOCTTY);
    if (serialFd < 0) {
        perror("Error opening serial port");
        exit(-1); 
    }

    struct termios oldSettings, newSettings;

    // Get the current serial port settings
    if (tcgetattr(serialFd, &oldSettings) != 0) {
        perror("Failed to get serial port attributes");
        close(serialFd);
        exit(-1);
    }

    // Configure the new settings
    bzero(&newSettings, sizeof(newSettings));
    newSettings.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newSettings.c_iflag = IGNPAR;
    newSettings.c_oflag = 0;
    newSettings.c_lflag = 0;
    newSettings.c_cc[VTIME] = 0;   /* inter-character timer unused */
    newSettings.c_cc[VMIN] = 0;    /* non-blocking read */

    // Clear the serial port buffer
    tcflush(serialFd, TCIOFLUSH);

    // Apply the new settings
    if (tcsetattr(serialFd, TCSANOW, &newSettings) != 0) {
        perror("Failed to set serial port attributes");
        close(serialFd);
        exit(-1);
    }

    return serialFd;
}
void alarmHandler(int signal) {
    alarmSignaled = TRUE;
    alarmCount++;
}

int llwrite(int descriptor, const unsigned char *dataBuffer, int bufferSize) {
    int packetLength = 6 + bufferSize;
    unsigned char *packet = (unsigned char *) malloc(packetLength);
    packet[0] = FLAG;
    packet[1] = A_ER;
    packet[2] = C_CONTROL(tramaTx);
    packet[3] = packet[1] ^ packet[2];
    memcpy(packet + 4, dataBuffer, bufferSize);

    unsigned char bccCheck = dataBuffer[0];
    for (unsigned int i = 1; i < bufferSize; i++) {
        bccCheck ^= dataBuffer[i];
    }

    int packetIdx = 4;
    for (unsigned int i = 0; i < bufferSize; i++) {
        if (dataBuffer[i] == FLAG || dataBuffer[i] == ESC) {
            packet = realloc(packet, ++packetLength);
            packet[packetIdx++] = ESC;
        }
        packet[packetIdx++] = dataBuffer[i];
    }
    packet[packetIdx++] = bccCheck;
    packet[packetIdx++] = FLAG;

    int result = primaryTransmitter(descriptor, packet, packetIdx);

    free(packet);
    return result;
}

unsigned char secondaryReceiver(int descriptor) {
    while (alarmSignaled == FALSE) {
        unsigned char response = readControlFrame(descriptor);
        
        if (!response) {
            continue;
        }
        else if (response == C_REJECTION(0) || response == C_REJECTION(1) || 
                 response == C_ACKNOWLEDGE(0) || response == C_ACKNOWLEDGE(1)) {
            return response;
        }
    }
    return 0; // Return 0 if timeout or an invalid response.
}

int primaryTransmitter(int descriptor, unsigned char *packet, int packetIdx) {
    int currentAttempt = 0;
    int hasRejections = 0, isAccepted = 0;

    while (currentAttempt < retransmitions) { 
        alarmSignaled = FALSE;
        alarm(timeout);
        hasRejections = 0;
        isAccepted = 0;
        
        write(descriptor, packet, packetIdx);
        unsigned char response = secondaryReceiver(descriptor);
        
        if (response == C_REJECTION(0) || response == C_REJECTION(1)) {
            hasRejections = 1;
        } else if (response == C_ACKNOWLEDGE(0) || response == C_ACKNOWLEDGE(1)) {
            isAccepted = 1;
            tramaTx = (tramaTx + 1) % 2;
        }

        if (isAccepted) break;
        currentAttempt++;
    }

    if (isAccepted) return packetIdx;
    else {
        llclose(descriptor);
        return -1;
    }
}


int llread(int fd, unsigned char *dataPacket) {
    unsigned char currentByte, controlField;
    int dataIndex = 0;
    LinkLayerState currentState = START;

    while (currentState != STOP_R) {
        if (read(fd, &currentByte, 1) > 0) {
            switch (currentState) {
                case START:
                    currentState = (currentByte == FLAG) ? FLAG_RCV : START;
                    break;
                case FLAG_RCV:
                    if (currentByte == A_ER)
                        currentState = ADDRESS_RCV;
                    else if (currentByte != FLAG)
                        currentState = START;
                    break;
                case ADDRESS_RCV:
                    if (currentByte == C_CONTROL(0) || currentByte == C_CONTROL(1)) {
                        controlField = currentByte;
                        currentState = CONTROL_RCV;
                    } else if (currentByte == FLAG) {
                        currentState = FLAG_RCV;
                    } else if (currentByte == C_DISC) {
                        sendSupervisionFrame(fd, A_RE, C_DISC);
                        return 0;
                    } else {
                        currentState = START;
                    }
                    break;
                case CONTROL_RCV:
                    currentState = (currentByte == (A_ER ^ controlField)) ? READING_DATA :
                                   (currentByte == FLAG) ? FLAG_RCV : START;
                    break;
                case READING_DATA:
                    if (currentByte == ESC)
                        currentState = DATA_FOUND_ESC;
                    else if (currentByte == FLAG) {
                        unsigned char bcc2 = dataPacket[dataIndex - 1];
                        dataIndex--;
                        dataPacket[dataIndex] = '\0';
                        unsigned char checksum = dataPacket[0];

                        for (unsigned int j = 1; j < dataIndex; j++)
                            checksum ^= dataPacket[j];

                        if (bcc2 == checksum) {
                            currentState = STOP_R;
                            sendSupervisionFrame(fd, A_RE, C_ACKNOWLEDGE(tramaRx));
                            tramaRx = (tramaRx + 1) % 2;
                            return dataIndex;
                        } else {
                            printf("Error: retransmission\n");
                            sendSupervisionFrame(fd, A_RE, C_REJECTION(tramaRx));
                            return -1;
                        }
                    } else {
                        dataPacket[dataIndex++] = currentByte;
                    }
                    break;
                case DATA_FOUND_ESC:
                    currentState = READING_DATA;
                    dataPacket[dataIndex++] = (currentByte == ESC || currentByte == FLAG) ? currentByte : ESC ^ currentByte;
                    break;
                default:
                    break;
            }
        }
    }
    return -1;
}

int llclose(int fd) {
    LinkLayerState state = START;
    unsigned char byte;
    (void) signal(SIGALRM, alarmHandler);

    while (retransmitions != 0 && state != STOP_R) {
        sendSupervisionFrame(fd, A_ER, C_DISC);
        alarm(timeout);
        alarmSignaled = FALSE;

        while (!alarmSignaled && state != STOP_R) {
            int bytesRead = read(fd, &byte, 1);
            if (bytesRead > 0) {
                processReceivedByte(&state, byte);
            }
        }
        retransmitions--;
    }

    if (state != STOP_R) return -1;
    sendSupervisionFrame(fd, A_ER, C_UA);
    return close(fd);
}

void processReceivedByte(LinkLayerState* state, unsigned char byte) {
    switch (*state) {
        case START:
            if (byte == FLAG) *state = FLAG_RCV;
            break;
        case FLAG_RCV:
            *state = (byte == A_RE) ? ADDRESS_RCV : (byte != FLAG) ? START : FLAG_RCV;
            break;
        case ADDRESS_RCV:
            if (byte == C_DISC) *state = CONTROL_RCV;
            else if (byte == FLAG) *state = FLAG_RCV;
            else *state = START;
            break;
        case CONTROL_RCV:
            *state = (byte == (A_RE ^ C_DISC)) ? BCC1_OK :
                     (byte == FLAG) ? FLAG_RCV : START;
            break;
        case BCC1_OK:
            *state = (byte == FLAG) ? STOP_R : START;
            break;
        default: 
            break;
    }
}

unsigned char readControlFrame(int portDescriptor) {
    unsigned char receivedByte, controlField = 0;
    LinkLayerState currentState = START;

    while (currentState != STOP_R && !alarmSignaled) {
        if (read(portDescriptor, &receivedByte, 1) > 0) {
            switch (currentState) {
                case START:
                    currentState = (receivedByte == FLAG) ? FLAG_RCV : START;
                    break;
                case FLAG_RCV:
                    currentState = (receivedByte == A_RE) ? ADDRESS_RCV :
                                  (receivedByte != FLAG) ? START : FLAG_RCV;
                    break;
                case ADDRESS_RCV:
                    if (receivedByte == C_ACKNOWLEDGE(0) || receivedByte == C_ACKNOWLEDGE(1) ||
                        receivedByte == C_REJECTION(0) || receivedByte == C_REJECTION(1) || 
                        receivedByte == C_DISC) {
                        currentState = CONTROL_RCV;
                        controlField = receivedByte;
                    } else if (receivedByte == FLAG) {
                        currentState = FLAG_RCV;
                    } else {
                        currentState = START;
                    }
                    break;
                case CONTROL_RCV:
                    currentState = (receivedByte == (A_RE ^ controlField)) ? BCC1_OK :
                                  (receivedByte == FLAG) ? FLAG_RCV : START;
                    break;
                case BCC1_OK:
                    currentState = (receivedByte == FLAG) ? STOP_R : START;
                    break;
                default:
                    break;
            }
        }
    }
    return controlField;
}



int sendSupervisionFrame(int portDescriptor, unsigned char addressField, unsigned char controlField) {
    unsigned char supervisionFrame[5] = {
        FLAG,
        addressField,
        controlField,
        addressField ^ controlField,
        FLAG
    };
    return write(portDescriptor, supervisionFrame, sizeof(supervisionFrame));
}
