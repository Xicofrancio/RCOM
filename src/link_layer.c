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
/*
int llopen(LinkLayer connectionParameters) {
    
    LinkLayerState state = START;
    int fd = connection(connectionParameters.serialPort);
    if (fd < 0) return -1;

    unsigned char byte;
    timeout = connectionParameters.timeout;
    retransmitions = connectionParameters.nRetransmissions;
    switch (connectionParameters.role) {

        case LlTx: {

            (void) signal(SIGALRM, alarmHandler);
            while (connectionParameters.nRetransmissions != 0 && state != STOP_R) {
                
                sendSupervisionFrame(fd, A_ER, C_SET);
                alarm(connectionParameters.timeout);
                alarmSignaled = FALSE;
                
                while (alarmSignaled == FALSE && state != STOP_R) {
                    if (read(fd, &byte, 1) > 0) {
                        switch (state) {
                            case START:
                                if (byte == FLAG) state = FLAG_RCV;
                                break;
                            case FLAG_RCV:
                                if (byte == A_RE) state = ADDRESS_RCV;
                                else if (byte != FLAG) state = START;
                                break;
                            case ADDRESS_RCV:
                                if (byte == C_UA) state = CONTROL_RCV;
                                else if (byte == FLAG) state = FLAG_RCV;
                                else state = START;
                                break;
                            case CONTROL_RCV:
                                if (byte == (A_RE ^ C_UA)) state = BCC1_OK;
                                else if (byte == FLAG) state = FLAG_RCV;
                                else state = START;
                                break;
                            case BCC1_OK:
                                if (byte == FLAG) state = STOP_R;
                                else state = START;
                                break;
                            default: 
                                break;
                        }
                    }
                } 
                connectionParameters.nRetransmissions--;
            }
            if (state != STOP_R) return -1;
            break;  
        }

        case LlRx: {

            while (state != STOP_R) {
                if (read(fd, &byte, 1) > 0) {
                    switch (state) {
                        case START:
                            if (byte == FLAG) state = FLAG_RCV;
                            break;
                        case FLAG_RCV:
                            if (byte == A_ER) state = ADDRESS_RCV;
                            else if (byte != FLAG) state = START;
                            break;
                        case ADDRESS_RCV:
                            if (byte == C_SET) state = CONTROL_RCV;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case CONTROL_RCV:
                            if (byte == (A_ER ^ C_SET)) state = BCC1_OK;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case BCC1_OK:
                            if (byte == FLAG) state = STOP_R;
                            else state = START;
                            break;
                        default: 
                            break;
                    }
                }
            }  
            sendSupervisionFrame(fd, A_RE, C_UA);
            break; 
        }
        default:
            return -1;
            break;
    }
    return fd;
}
*/
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


int llread(int fd, unsigned char *packet) {

    unsigned char byte, cField;
    int i = 0;
    LinkLayerState state = START;

    while (state != STOP_R) {  
        if (read(fd, &byte, 1) > 0) {
            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == A_ER) state = ADDRESS_RCV;
                    else if (byte != FLAG) state = START;
                    break;
                case ADDRESS_RCV:
                    if (byte == C_CONTROL(0) || byte == C_CONTROL(1)){
                        state = CONTROL_RCV;
                        cField = byte;   
                    }
                    else if (byte == FLAG) state = FLAG_RCV;
                    else if (byte == C_DISC) {
                        sendSupervisionFrame(fd, A_RE, C_DISC);
                        return 0;
                    }
                    else state = START;
                    break;
                case CONTROL_RCV:
                    if (byte == (A_ER ^ cField)) state = READING_DATA;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case READING_DATA:
                    if (byte == ESC) state = DATA_FOUND_ESC;
                    else if (byte == FLAG){
                        unsigned char bcc2 = packet[i-1];
                        i--;
                        packet[i] = '\0';
                        unsigned char acc = packet[0];

                        for (unsigned int j = 1; j < i; j++)
                            acc ^= packet[j];

                        if (bcc2 == acc){
                            state = STOP_R;
                            sendSupervisionFrame(fd, A_RE, C_ACKNOWLEDGE(tramaRx));
                            tramaRx = (tramaRx + 1)%2;
                            return i; 
                        }
                        else{
                            printf("Error: retransmition\n");
                            sendSupervisionFrame(fd, A_RE, C_REJECTION(tramaRx));
                            return -1;
                        };

                    }
                    else{
                        packet[i++] = byte;
                    }
                    break;
                case DATA_FOUND_ESC:
                    state = READING_DATA;
                    if (byte == ESC || byte == FLAG) packet[i++] = byte;
                    else{
                        packet[i++] = ESC;
                        packet[i++] = byte;
                    }
                    break;
                default: 
                    break;
            }
        }
    }
    return -1;
}

int llclose(int fd){

    LinkLayerState state = START;
    unsigned char byte;
    (void) signal(SIGALRM, alarmHandler);
    
    while (retransmitions != 0 && state != STOP_R) {
                
        sendSupervisionFrame(fd, A_ER, C_DISC);
        alarm(timeout);
        alarmSignaled = FALSE;
                
        while (alarmSignaled == FALSE && state != STOP_R) {
            if (read(fd, &byte, 1) > 0) {
                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == A_RE) state = ADDRESS_RCV;
                        else if (byte != FLAG) state = START;
                        break;
                    case ADDRESS_RCV:
                        if (byte == C_DISC) state = CONTROL_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case CONTROL_RCV:
                        if (byte == (A_RE ^ C_DISC)) state = BCC1_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC1_OK:
                        if (byte == FLAG) state = STOP_R;
                        else state = START;
                        break;
                    default: 
                        break;
                }
            }
        } 
        retransmitions--;
    }

    if (state != STOP_R) return -1;
    sendSupervisionFrame(fd, A_ER, C_UA);
    return close(fd);
}

unsigned char readControlFrame(int fd){

    unsigned char byte, cField = 0;
    LinkLayerState state = START;
    
    while (state != STOP_R && alarmSignaled == FALSE) {  
        if (read(fd, &byte, 1) > 0 || 1) {
            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == A_RE) state = ADDRESS_RCV;
                    else if (byte != FLAG) state = START;
                    break;
                case ADDRESS_RCV:
                    if (byte == C_ACKNOWLEDGE(0) || byte == C_ACKNOWLEDGE(1) || byte == C_REJECTION(0) || byte == C_REJECTION(1) || byte == C_DISC){
                        state = CONTROL_RCV;
                        cField = byte;   
                    }
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case CONTROL_RCV:
                    if (byte == (A_RE ^ cField)) state = BCC1_OK;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case BCC1_OK:
                    if (byte == FLAG){
                        state = STOP_R;
                    }
                    else state = START;
                    break;
                default: 
                    break;
            }
        } 
    } 
    return cField;
}

int sendSupervisionFrame(int fd, unsigned char A, unsigned char C){
    unsigned char FRAME[5] = {FLAG, A, C, A ^ C, FLAG};
    return write(fd, FRAME, 5);
}