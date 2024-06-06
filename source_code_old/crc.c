#include <stdio.h>
#include <string.h>

void swuart_calcCRC(unsigned char* datagram, unsigned char datagramLength)
{
    int i,j;
    unsigned char* crc = datagram + (datagramLength-1); // CRC located in last byte of message
    unsigned char currentByte;
    *crc = 0;
    for (i=0; i<(datagramLength-1); i++) { // Execute for all bytes of a message
        currentByte = datagram[i]; // Retrieve a byte to be sent from Array
        for (j=0; j<8; j++) {
            if ((*crc >> 7) ^ (currentByte&0x01)){ // update CRC based result of XOR operation
                *crc = (*crc << 1) ^ 0x07;
            }
            else{
                *crc = (*crc << 1);
            }
            currentByte = currentByte >> 1;
        } // for CRC bit
    } // for message byte
}

void hex_to_unsigned_char(const char *hex_string, unsigned char *output, size_t output_size) {
    size_t hex_len = strlen(hex_string);
    
    // Check if hex string length is even and not longer than output size
    if (hex_len % 2 != 0 || hex_len / 2 > output_size) {
        printf("Invalid hex string or output size\n");
        return;
    }
    
    // Convert each pair of hex characters to unsigned char
    for (size_t i = 0; i < hex_len; i += 2) {
        sscanf(hex_string + i, "%2hhX", &output[i / 2]);
    }
}

int main(int argc, char *argv[]){
    const char *hexstring = argv[1];
    unsigned char datagram[8];

    hex_to_unsigned_char(hexstring, datagram, sizeof(datagram));

    printf("Output without crc: ");
    for (int i = 0; i < sizeof(datagram); i++){
        printf("%02X", datagram[i]);
    }
    printf("\n");

    swuart_calcCRC(datagram, sizeof(datagram)/sizeof(unsigned char));

    printf("Output with crc: ");
    for (int i = 0; i < sizeof(datagram); i++){
        printf("%02X", datagram[i]);
    }
    printf("\n");

    return 0;
}