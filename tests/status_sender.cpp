#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstdio>
#include <cstdint> // uint8_t
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <string>

std::string msg1 = "Hello there!\n";
char magic[] = {0x41, 0x54, 0x53, 0x46};

int main()
{
    int sock;
    struct sockaddr_in addr;

    sock = socket(AF_INET, SOCK_DGRAM, 0); // create socket for datagrams (UDP) in Internet address space
    if(sock < 0)
    {
        perror("socket creation");
        exit(1);
    }

    addr.sin_family = AF_INET; // address space - Internet
    addr.sin_port = htons(0x4653); // convert Port number from host byte order to network byte order
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // convert Localhost address from host byte order to network byte order
    sendto(sock, &msg1[0], msg1.size(), 0,
           (struct sockaddr *)&addr, sizeof(addr));

//    connect(sock, (struct sockaddr *)&addr, sizeof(addr)); // implicitly bind socket with particular address
//    send(sock, msg2, sizeof(msg2), 0);

    timespec waittime;
    long int period_ = 100000000;
    waittime.tv_sec = period_/1000000000; /* seconds */
    waittime.tv_nsec = period_%1000000000; /* nanoseconds */

    /* initialize next_arrival_time_ */
    long int periods_micro=period_/1000;
    timeval now, next_arrival_time_;
    gettimeofday(&now, NULL);
    next_arrival_time_.tv_sec = now.tv_sec + periods_micro/1000000;
    next_arrival_time_.tv_usec = now.tv_usec + periods_micro%1000000;

    for (int i = 0; i < 100; i++){
        /*
         *  USING STRING TO COMBINE UDP MSG
         */
        std::string status;

        // Combine msg starting with magic word
        status.append(magic, sizeof(char)*4);

        // Timestamp
        gettimeofday(&now, NULL);
//        std::cout << "Time: " << now.tv_sec << " sec, " << now.tv_usec/1000 << " ms \n";
//        std::cout << "Timeval field sizes: sizeof(tv_sec) = " << sizeof(now.tv_sec) << ", sizeof(tv_usec) = " << sizeof(now.tv_usec) << "\n";
        uint32_t timestamp = (now.tv_sec - 1531000000) * 1000 + now.tv_usec/1000;
//        std::cout << "Combined timestamp: " << timestamp << " ms\n";
//        std::cout << "Time after conversion: " << timestamp/1000 << " sec, " << timestamp%1000 << " ms \n";
        uint32_t timestamp_net = htonl(timestamp);
        status.append((const char *) &timestamp_net, sizeof(uint32_t));

        // System count
        uint8_t sc = i; // Unsigned char no need to change byte order for network transfer
        status.append((const char *) &sc, sizeof(uint8_t));

        // Zero padding
        uint8_t padd = 0;
        status.append((const char *) &padd, sizeof(uint8_t));
        status.append((const char *) &padd, sizeof(uint8_t));
        status.append((const char *) &padd, sizeof(uint8_t));

        // BMS data
        uint8_t sid_bms = 1;
        status.append((const char *) &sid_bms, sizeof(uint8_t));
        uint8_t bms_state = 2; // running
        status.append((const char *) &bms_state, sizeof(uint8_t));
        uint8_t bms_charge = 200;
        status.append((const char *) &bms_charge, sizeof(uint8_t));
        uint16_t bms_current = 1234;
        uint16_t bms_current_net = htons(bms_current);
        status.append((const char *) &bms_current_net, sizeof(uint16_t));
        uint16_t bms_voltage = 2345;
        uint16_t bms_voltage_net = htons(bms_voltage);
        status.append((const char *) &bms_voltage_net, sizeof(uint16_t));
        uint8_t bms_temp_h = 40;
        status.append((const char *) &bms_temp_h, sizeof(uint8_t));
        uint8_t bms_temp_l = 60;
        status.append((const char *) &bms_temp_l, sizeof(uint8_t));



        std::cout << "Resulting string: " << status << "; size = " << status.size() << "\n";

        // Send data
        sendto(sock, &status[0], status.size(), 0, (struct sockaddr *)&addr, sizeof(addr));

        gettimeofday(&now, NULL);

        /* after execution, compute the time to the beginning of the next period */
        long int timetowait= 1000*((next_arrival_time_.tv_sec - now.tv_sec)*1000000
                                   +(next_arrival_time_.tv_usec-now.tv_usec));

        if (timetowait < 0)
        {
            std::cout << "\n Doesn't fit in period" << std::endl;
        }
        else
        {
            waittime.tv_sec = timetowait/1000000000;
            waittime.tv_nsec = timetowait%1000000000;

            /* suspend the task until the beginning of the next period */
            nanosleep(&waittime, NULL);
        }

        /* the task is ready: set the next arrival time */
        next_arrival_time_.tv_sec = next_arrival_time_.tv_sec +
                                    periods_micro/1000000;
        next_arrival_time_.tv_usec = next_arrival_time_.tv_usec +
                                     periods_micro%1000000;

    }

    close(sock);

    return 0;
}