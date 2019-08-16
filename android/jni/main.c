#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <strings.h>
#include <time.h>

/* threads */
#include <sched.h>
#include <pthread.h>

/* sockets */
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>

/* serial communications */
#include <termios.h>
#include <fcntl.h>
#include <sys/stat.h>

#ifndef NOT845
#include <sys/ioctl.h>
#include <asm-generic/ioctls.h>

#define TIOCPMGET 0x5441 /* PM get */
#define TIOCPMPUT 0x5442 /* PM put */
#define TIOCPMACT 0x5443 /* PM is active */
#endif

int debug = 0, verbose = 0, n3 = 0;

int set_blocking_mode(int fd, int blocking) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (blocking) { flags &= ~O_NONBLOCK; }
    else { flags |= O_NONBLOCK; }
    fcntl(fd, F_SETFL, flags);
    return 0;
}

void print_buffer(const char *msg, uint8_t *p, int n) {
    printf("%s\n", msg);
    int k;
    for (k = 0; k < n; ++k) {
        printf("%2.2x ", p[k]);
        if ((k & 7) == 7) { printf("\n"); }
    }
}

struct termios old_settings;
int initialize_android_serial_port(const char *dev, int baud_rate, int canonical, int parity, int min_chars) {

    printf("initialize_serial_port (android)\n");

    int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    if(fd < 0) { return fd; }

    usleep(100 * 1000);
    ioctl(fd, TIOCPMGET);  /* PM Get */
    usleep(100 * 1000);

    fcntl(fd, F_SETFL, 0);
    struct termios *settings;

    memset(&old_settings, 0, sizeof(old_settings));
    tcgetattr(fd, &old_settings);

    struct termios new_settings;
    memset(&new_settings, 0, sizeof(new_settings));
    new_settings = old_settings; /* start here */

    /* effect new settings */
    settings = &new_settings;
    cfmakeraw(settings);
    settings->c_cflag &= ~(CSIZE | CRTSCTS | CSTOPB | PARENB); /* no parity, one stop bit, no cts/rts, clear size */
    settings->c_cflag |= CS8; /* eight bits */
    settings->c_cflag |= (CLOCAL | CREAD); /* ignore carrier detect. enable receiver */
    settings->c_iflag &= ~(IXON | IXOFF | IXANY | IGNPAR | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    settings->c_iflag |= ( IGNPAR | IGNBRK);
    settings->c_lflag &= ~(ECHOK | ECHOCTL | ECHOKE);
    if (canonical) { settings->c_lflag |= ICANON; } /* set canonical */
    else { settings->c_lflag &= ~ICANON; } /* or clear it */
    settings->c_oflag &= ~(OPOST | ONLCR);
    settings->c_cc[VMIN] = 0;
    settings->c_cc[VTIME] = 5; /* 200ms timeout */

    if (baud_rate == 115200) {
        cfsetispeed(settings, B115200);
        cfsetospeed(settings, B115200);
    } else if (baud_rate == 9600) {
        cfsetispeed(settings, B9600);
        cfsetospeed(settings, B9600);
    } else {
        return 0;
    }

    tcsetattr(fd, TCSANOW, settings); /* apply settings */

    tcflush(fd, TCIOFLUSH);

    return fd;
}

/* parity = 0 (no parity), = 1 odd parity, = 2 even parity */
int initialize_serial_port(const char *dev, int baud_rate, int canonical, int parity, int min_chars) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    // int fd = open(dev, O_RDWR | O_NOCTTY);
    // int fd = open(dev, O_WRONLY | O_NOCTTY);
    if(fd < 0) { return fd; }
    fcntl(fd, F_SETFL, 0);
    struct termios *settings, current_settings;

    memset(&current_settings, 0, sizeof(current_settings));
    tcgetattr(fd, &current_settings);

    /* effect new settings */
    settings = &current_settings;
    cfmakeraw(settings);
    if (parity == 0) {
        settings->c_cflag &= ~(CSIZE | CRTSCTS | CSTOPB | PARENB); /* no parity, one stop bit, no cts/rts, clear size */
        settings->c_cflag |= CS8; /* eight bits */
    } else if (parity == 1) {
        settings->c_cflag &= ~(CSIZE | CRTSCTS | CSTOPB); /* no parity, one stop bit, no cts/rts, clear size */
        settings->c_cflag |= (CS8 | PARENB | PARODD); /* eight bits, odd parity */
    } else if (parity == 2) {
        settings->c_cflag &= ~(CSIZE | CRTSCTS | CSTOPB | PARODD); /* no parity, one stop bit, no cts/rts, clear size */
        settings->c_cflag |= (CS8 | PARENB); /* eight bits, odd parity is clear for even parity */
    }
    settings->c_cflag |= (CLOCAL | CREAD); /* ignore carrier detect. enable receiver */
    settings->c_iflag &= ~(IXON | IXOFF | IXANY | IGNPAR | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    settings->c_iflag |= ( IGNPAR | IGNBRK);
    settings->c_lflag &= ~(ECHOK | ECHOCTL | ECHOKE);
    if (canonical) { settings->c_lflag |= ICANON; } /* set canonical */
    else { settings->c_lflag &= ~ICANON; } /* or clear it */
    settings->c_oflag &= ~(OPOST | ONLCR);
    settings->c_cc[VMIN] = min_chars;
    settings->c_cc[VTIME] = 1;

    if (baud_rate == 115200) {
        cfsetispeed(settings, B115200);
        cfsetospeed(settings, B115200);
    } else if (baud_rate == 9600) {
        cfsetispeed(settings, B9600);
        cfsetospeed(settings, B9600);
    } else {
        return 0;
    }

    tcsetattr(fd, TCSANOW, settings); /* apply settings */
    tcflush(fd, TCIOFLUSH);

    return fd;
}

typedef struct LooperArgsPreamble {
    int *arm;
    int *run;
    int *thread_run;
} LooperArgsPreamble;

typedef struct Queue {
    uint8_t *buff;
    unsigned int head, tail, mask;
    unsigned long int cntr;
} Queue;

typedef struct Pool {
    uint8_t **buff;
    unsigned int *length;
    unsigned int head;
    unsigned int tail;
    unsigned int mask;
    unsigned int buff_size;
} Pool;

void initialize_pool(Pool *pool, int buffs, int buff_size) {
    bzero(pool, sizeof(Pool));
    pool->buff = (uint8_t **) malloc(buffs * sizeof(uint8_t *));
    pool->mask = buffs - 1;
    pool->length = (unsigned int *) malloc(buffs * sizeof(unsigned int));
    for (int i = 0; i < buffs; ++i) {
        pool->buff[i] = (uint8_t *) malloc(buff_size);
    }
    pool->buff_size = buff_size;
}

void initialize_queue(Queue *queue, int length) {
    bzero(queue, sizeof(Queue));
    queue->buff = (uint8_t *) malloc(length);
    queue->mask = length - 1;
}

int open_uart(const char *dev_name, int baud_rate) {
    int canonical = 0;
    int parity = 0; /* none */
    int min_chars = 8;
    if (n3) {
        return initialize_android_serial_port(dev_name, baud_rate, canonical, parity, min_chars);
    } else {
        return initialize_serial_port(dev_name, baud_rate, canonical, parity, min_chars);
    }
}

typedef struct ServerArgs {
    pthread_t *tid;
    int sock_fd;
    int conn_fd;
    int port;
    LooperArgsPreamble preamble;
    int state;
    Pool *rx_pool;
    Pool *tx_pool;
    struct sockaddr_in cli;
    struct sockaddr_in servaddr;
} ServerArgs;

enum {
    ServerStateIdle = 0,
    ServerStateConnected,
    ServerStates
};

typedef struct TxLooperArgs {
    pthread_t *tid;
    Pool *pool;
    LooperArgsPreamble preamble;
    int fd;
    unsigned int verbose, debug, format;
} TxLooperArgs;

void *tx_looper(void *ext) {
    TxLooperArgs *args = (TxLooperArgs *) ext;
    LooperArgsPreamble *preamble = &args->preamble;
    Pool *pool = args->pool;
    while (*preamble->thread_run) {
        while (*preamble->arm == 0) { usleep(1000); }
        while (*preamble->run) {
            while (pool->head != pool->tail) {
                if (pool->length[pool->tail] && args->fd) {
                    int remaining = pool->length[pool->tail];
                    while (remaining > 0) {
                        int n_bytes = write(args->fd, pool->buff[pool->tail], remaining);
                        remaining = remaining - n_bytes;
                    }
                    pool->length[pool->tail] = 0; /* clear once done */
                    pool->tail = (pool->tail + 1) & pool->mask;
                } else {
                    usleep(1000); /* micronap */
                }
            }
        }
    }
    return NULL;
}

typedef struct RxLooperArgs {
    pthread_t *tid;
    LooperArgsPreamble preamble;
    int fd;
    unsigned int verbose, debug, format;
    Pool *pool;
} RxLooperArgs;

enum {
    DISPLAY_FORMAT_ASCII = 0,
    DISPLAY_FORMAT_HEX,
    DISPLAY_FORMATS
};

void *rx_looper(void *ext) {
    RxLooperArgs *args = (RxLooperArgs *) ext;
    Pool *pool = args->pool;
    LooperArgsPreamble *preamble = &args->preamble;
    int last_error = 0;
    time_t now, holdoff_timeout = 0;
    while (*preamble->thread_run) {
        while (*preamble->arm == 0) { usleep(1000); }
        while (*preamble->run) {
            unsigned int new_head = (pool->head + 1) & pool->mask;
            if (new_head != pool->tail) {
                uint8_t *ptr = pool->buff[pool->head];
                ssize_t n_read = read(args->fd, pool->buff[pool->head], pool->buff_size);
                if (n_read > 0) {
                    print_buffer("FROM DEVICE:", ptr, n_read);
                    printf("read %ld bytes from device %d\n", n_read, args->fd); /* TODO */
                    pool->length[pool->head] = n_read;
                    if (n_read != pool->buff_size) { pool->buff[pool->head][n_read] = 0; } /* null-terminate */
                    pool->head = new_head;
                    printf("new head = %d. pool = %p\n", pool->head, pool);
//                    if ((n_read > 0) && args->verbose) {
//                        printf("\n");
//                        for (int i = 0; i < n_read; ++i) {
//                            uint8_t byte = ptr[i];
//                            if (args->format == DISPLAY_FORMAT_ASCII) { printf("%c", byte); }
//                            else if (args->format == DISPLAY_FORMAT_HEX) { printf("%2.2x ", byte); }
//                        }
//                        printf("\n");
//                    }
                } else if (n_read == -1) {
                    now = time(0);
                    int holdoff_ok = (holdoff_timeout == 0) || (now > holdoff_timeout);
                    if (errno != last_error) {
                        last_error = errno;
                    }
                    if (holdoff_ok) {
                        perror("input device read error");
                        printf("error reading from %d\n", args->fd);
                        holdoff_timeout = now + 1; /* once a second max */
                    }
                }
            }
            usleep(1000);
        }
    }
    return NULL;
}

void *server_loop(void *ext) {
    ServerArgs *args = (ServerArgs *) ext;
    LooperArgsPreamble *preamble = &args->preamble;
    args->state = ServerStateIdle;
    while (*preamble->thread_run) {
        while (*preamble->arm == 0) { usleep(1000); }
        while (*preamble->run) {
            switch (args->state) {

            case ServerStateIdle: {
                socklen_t len = sizeof(args->cli);
                args->conn_fd = accept(args->sock_fd, (struct sockaddr *) &args->cli, &len);
                set_blocking_mode(args->conn_fd, 0);
                if (args->conn_fd > 0) {
                    printf("accepted connection on port %d. connection = %d\n", args->port, args->conn_fd);
                    args->state = ServerStateConnected;
                } else if (args->conn_fd < 0) {
                    if (errno != EAGAIN) {
                        printf("connection %d rejected incoming request on port %d. reason = %d\n", args->sock_fd,
                               args->port, errno);
                    }
                }
                break;
            }

            case ServerStateConnected: {
                /* from socket to device. socket produces into head; device consumes from tail */
                Pool *pool = args->tx_pool;
                uint8_t *ptr = pool->buff[pool->head];
                int n_bytes = read(args->conn_fd, pool->buff[pool->head], pool->buff_size);
                if (n_bytes > 0) {
                    pool->length[pool->head] = n_bytes;
                    pool->head = (pool->head + 1) & pool->mask;
                    print_buffer("TO DEVICE:", ptr, n_bytes);
                }

                /* from device to socket. device produces into head; socket consumes from tail. */
                pool = args->rx_pool;
                if (pool->head != pool->tail) {
                    int remaining = pool->length[pool->tail];
                    print_buffer("FROM DEVICE", ptr, remaining);
                    while (remaining) {
                        n_bytes = write(args->conn_fd, pool->buff[pool->tail], remaining);
                        remaining = remaining - n_bytes;
                    }
                    pool->tail = (pool->tail + 1) & pool->mask;
                }
                break;
            }
            }
            usleep(1000);
        }
    }
    return NULL;
}

typedef struct ClientArgs {
    pthread_t *tid;
    int port;
    int sock_fd;
    LooperArgsPreamble preamble;
    char ip_addr[128];
    Pool *rx_pool;
    Pool *tx_pool;
    struct sockaddr_in servaddr;
} ClientArgs;

void *client_loop(void *ext) {
    ClientArgs *args = (ClientArgs *) ext;
    LooperArgsPreamble *preamble = &args->preamble;

    if (connect(args->sock_fd, (struct sockaddr *) &args->servaddr, sizeof(args->servaddr)) != 0) {
        printf("boo hoo\n");
        exit(0);
    } else {
        printf("yay\n");
    }

    set_blocking_mode(args->sock_fd, 0);

    while (*preamble->thread_run) {
        while (*preamble->arm == 0) { usleep(1000); }
        while (*preamble->run) {
            /* from socket to device. socket produces into head; device consumes from tail */
            Pool *pool = args->tx_pool;
            uint8_t *ptr = pool->buff[pool->head];
            int n_bytes = read(args->sock_fd, pool->buff[pool->head], pool->buff_size);
            if (n_bytes > 0) {
                pool->length[pool->head] = n_bytes;
                pool->head = (pool->head + 1) & pool->mask;
                print_buffer("TO DEVICE:", ptr, n_bytes);
            }

            /* from device to socket. device produces into head; socket consumes from tail. */
            pool = args->rx_pool;
            if (pool->head != pool->tail) {
                int remaining = pool->length[pool->tail];
                uint8_t *ptr = pool->buff[pool->tail];
                print_buffer("FROM DEVICE", ptr, remaining);
                // printf("received %d bytes for writing to %d\n", remaining, args->sock_fd);
                while (remaining) {
                    n_bytes = write(args->sock_fd, pool->buff[pool->tail], remaining);
                    remaining = remaining - n_bytes;
                }
                pool->tail = (pool->tail + 1) & pool->mask;
                // printf("head = %d. tail = %d. mask = %d\n", pool->head, pool->tail, pool->mask);
            }
            usleep(1000);
        }
    }
    return NULL;
}

int open_socket(int server_mode, const char *ip_addr, struct sockaddr_in *servaddr, int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd == -1) { return 0; }
    printf("socket %d successfully created\n", fd);
    bzero(servaddr, sizeof(struct sockaddr_in));
    servaddr->sin_family = AF_INET;
    servaddr->sin_port = htons(port);
    if (server_mode) {
        servaddr->sin_addr.s_addr = htonl(INADDR_ANY);
        if (bind(fd, (struct sockaddr *) servaddr, sizeof(struct sockaddr_in)) != 0) {
            printf("socket failed to bind\n");
            return 0;
        }
        printf("socket bind success\n");
        const int backlog = 5;
        if ((listen(fd, backlog)) != 0) {
            printf("server listen failed\n");
            return 0;
        }
    } else {
        servaddr->sin_addr.s_addr = inet_addr(ip_addr);
    }
    return fd;
}

int main(int argc, char **argv)
{
#define QueueSize (512)
#define PoolSize (64)

    write(1, "hello, world\n", 13);

    Pool tx_pool, rx_pool;
    initialize_pool(&tx_pool, PoolSize, QueueSize);
    initialize_pool(&rx_pool, PoolSize, QueueSize);
    pthread_t socket_thread, device_rx_thread, device_tx_thread;
    int thread_run = 1, arm = 1, run = 1;
    int server_mode = 0;
    ServerArgs *server_args = NULL;
    ClientArgs *client_args = NULL;
    RxLooperArgs *rx_looper_args = NULL;
    TxLooperArgs *tx_looper_args = NULL;
    LooperArgsPreamble *preamble;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-debug") == 0) {
            debug = 1;
        } else if (strcmp(argv[i], "-verbose") == 0) {
            verbose = 1;
        } else if (strcmp(argv[i], "-n3") == 0) {
            n3 = 1;
        } else if (strcmp(argv[i], "-file") == 0) {
            rx_looper_args = (RxLooperArgs *) malloc(sizeof(RxLooperArgs));
            bzero(rx_looper_args, sizeof(RxLooperArgs));
            rx_looper_args->fd = open(argv[++i], O_RDONLY, S_IRUSR);
            if (rx_looper_args->fd < 0) {
                printf("unable to open input file\n");
                return -1;
            }
            rx_looper_args->pool = &rx_pool;
            preamble = &rx_looper_args->preamble;
            preamble->run = &run;
            preamble->arm = &arm;
            preamble->thread_run = &thread_run;
            tx_looper_args = (TxLooperArgs *) malloc(sizeof(TxLooperArgs));
            bzero(tx_looper_args, sizeof(TxLooperArgs));
            tx_looper_args->fd = 1;
            tx_looper_args->pool = &tx_pool;
            preamble = &tx_looper_args->preamble;
            preamble->run = &run;
            preamble->arm = &arm;
            preamble->thread_run = &thread_run;
            pthread_create(&device_rx_thread, NULL, rx_looper, rx_looper_args);
            pthread_create(&device_tx_thread, NULL, tx_looper, tx_looper_args);
        } else if (strcmp(argv[i], "-console") == 0) {
            rx_looper_args = (RxLooperArgs *) malloc(sizeof(RxLooperArgs));
            bzero(rx_looper_args, sizeof(RxLooperArgs));
            rx_looper_args->tid = &device_rx_thread;
            rx_looper_args->fd = 0;
            rx_looper_args->pool = &rx_pool;
            preamble = &rx_looper_args->preamble;
            preamble->run = &run;
            preamble->arm = &arm;
            preamble->thread_run = &thread_run;
            tx_looper_args = (TxLooperArgs *) malloc(sizeof(TxLooperArgs));
            bzero(tx_looper_args, sizeof(TxLooperArgs));
            tx_looper_args->tid = &device_tx_thread;
            tx_looper_args->fd = 1;
            tx_looper_args->pool = &tx_pool;
            preamble = &tx_looper_args->preamble;
            preamble->run = &run;
            preamble->arm = &arm;
            preamble->thread_run = &thread_run;
            pthread_create(rx_looper_args->tid, NULL, rx_looper, rx_looper_args);
            pthread_create(tx_looper_args->tid, NULL, tx_looper, tx_looper_args);
        } else if (strcmp(argv[i], "-uart") == 0) {
            const char *device_name = argv[++i];
            int baud_rate = atoi(argv[++i]);

            rx_looper_args = (RxLooperArgs *) malloc(sizeof(RxLooperArgs));
            bzero(rx_looper_args, sizeof(RxLooperArgs));
            rx_looper_args->tid = &device_rx_thread;
            rx_looper_args->pool = &rx_pool;
            preamble = &rx_looper_args->preamble;
            preamble->run = &run;
            preamble->arm = &arm;
            preamble->thread_run = &thread_run;

            tx_looper_args = (TxLooperArgs *) malloc(sizeof(TxLooperArgs));
            bzero(tx_looper_args, sizeof(TxLooperArgs));
            tx_looper_args->tid = &device_tx_thread;
            tx_looper_args->pool = &tx_pool;
            preamble = &tx_looper_args->preamble;
            preamble->run = &run;
            preamble->arm = &arm;
            preamble->thread_run = &thread_run;

            rx_looper_args->fd = tx_looper_args->fd = open_uart(device_name, baud_rate);
            printf("opened device %d (uart)\n", rx_looper_args->fd);
            set_blocking_mode(rx_looper_args->fd, 0);
            pthread_create(rx_looper_args->tid, NULL, rx_looper, rx_looper_args);
            pthread_create(tx_looper_args->tid, NULL, tx_looper, tx_looper_args);
        } else if (strcmp(argv[i], "-server") == 0) {
            server_args = (ServerArgs *) malloc(sizeof(ServerArgs));
            bzero(server_args, sizeof(ServerArgs));
            server_args->tid = &socket_thread;
            server_args->port = atoi(argv[++i]);
            server_args->sock_fd = open_socket(1, NULL, &server_args->servaddr, server_args->port);
            server_args->rx_pool = &rx_pool;
            server_args->tx_pool = &tx_pool;
            preamble = &server_args->preamble;
            preamble->arm = &arm; /* make 0 to loop until armed. then set arm = 1 to drop into main loop */
            preamble->run = &run; /* run = 1 for main loop operation */
            preamble->thread_run = &thread_run; /* make 0 to quit */
            int err = pthread_create(server_args->tid, NULL, server_loop, (void *) server_args);
        } else if (strcmp(argv[i], "-client") == 0) {
            client_args = (ClientArgs *) malloc(sizeof(ClientArgs));
            bzero(client_args, sizeof(ClientArgs));
            strcpy(client_args->ip_addr, argv[++i]); /* TODO make safe */
            client_args->tid = &socket_thread;
            client_args->port = atoi(argv[++i]);
            client_args->sock_fd = open_socket(0, client_args->ip_addr, &client_args->servaddr, client_args->port);
            client_args->rx_pool = &rx_pool;
            client_args->tx_pool = &tx_pool;
            preamble = &client_args->preamble;
            preamble->arm = &arm;
            preamble->run = &run;
            preamble->thread_run = &thread_run;
            int err = pthread_create(client_args->tid, NULL, client_loop, (void *) client_args);
        }
    }

    while (thread_run) {
        usleep(1000);
    }
    thread_run = 0;
    run = 0;
    sleep(3); /* wait for threads to die */
    pthread_join(device_rx_thread, NULL);
    pthread_join(device_tx_thread, NULL);
    pthread_join(socket_thread, NULL);
    printf("good-byte\n");

    return 0;
}
