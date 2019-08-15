#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* threads */
#include <sched.h>
#include <pthread.h>

/* sockets */
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

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

uint8_t debug = 0, verbose = 0, n3 = 0;

int set_blocking_mode(int fd, int blocking) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (blocking) { flags &= ~O_NONBLOCK; }
    else { flags |= O_NONBLOCK; }
    fcntl(fd, F_SETFL, flags);
    return 0;
}

struct termios old_settings;
int initialize_n3_serial_port(const char *dev, int baud_rate, int canonical, int parity, int min_chars) {
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
} Pool;

void initialize_pool(Pool *pool, int buffs, int buff_size) {
    bzero(pool, sizeof(Pool));
    pool->buff = (uint8_t **) malloc(buffs * sizeof(uint8_t *));
    pool->mask = buffs - 1;
    pool->length = (unsigned int *) malloc(buffs * sizeof(unsigned int));
    for (int i = 0; i < buffs; ++i) {
        pool->buff[i] = (uint8_t *) malloc(buff_size);
    }
}

void initialize_queue(Queue *queue, int length) {
    bzero(queue, sizeof(Queue));
    queue->buff = (uint8_t *) malloc(length);
    queue->mask = length - 1;
}
//int get_xmit_buffer(Pool *pool, unsigned int id, uint8_t **buff, unsigned int **length);
//int flush_queue(Queue *q);

int open_uart(const char *dev_name, int baud_rate) {
    int canonical = 0;
    int parity = 0; /* none */
    int min_chars = 8;
    if (n3) {
        return initialize_n3_serial_port(dev_name, baud_rate, canonical, parity, min_chars);
    } else {
        return initialize_serial_port(dev_name, baud_rate, canonical, parity, min_chars);
    }
}

typedef struct ServerArgs {
    pthread_t *tid;
    int sock_fd;
    int conn_fd;
    int port;
    int *arm;
    int *run;
    int *thread_run;
    int state;
    struct sockaddr_in cli;
    Queue *rx_queue;
    Pool *emit_pool;
} ServerArgs;

enum {
    ServerStateIdle = 0,
    ServerStateConnected,
    ServerStates
};

typedef struct TxLooperArgs {
    pthread_t *tid;
    Pool *pool;
    unsigned int *run;
    int fd;
    unsigned int verbose, debug, format;
} TxLooperArgs;

void *tx_looper(void *ext) {
    TxLooperArgs *args = (TxLooperArgs *) ext;
    Pool *pool = args->pool;
    while (*args->run) {
        while (pool->head >= pool->tail) {
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

typedef struct RxLooperArgs {
    pthread_t *tid;
    Queue *queue;
    unsigned int *run;
    int fd;
    unsigned int verbose, debug, format;
    Queue *rx_queue;
    Pool *emit_pool;
} RxLooperArgs;

void *rx_looper(void *ext) {
    RxLooperArgs *args = (RxLooperArgs *) ext;
    Queue *q = args->queue;
    while (*args->run) {
        int fit = 0;
        if (q->head >= q->tail) {
            fit = 1 + q->mask - q->head;
        } else {
            fit = q->tail - q->head - 1;
        }
        if (fit) {
            int n_read = read(args->fd, &q->buff[q->head], fit);
            if ((n_read > 0) && args->verbose) {
                printf("\n");
                for (int i = 0; i < n_read; ++i) {
                    uint8_t byte = q->buff[(q->head + i) & q->mask];
                    if (args->format == FORMAT_ASCII) { printf("%c", byte); }
                    else if (args->format == FORMAT_HEX) { printf("%2.2x ", byte); }
                }
                if (n_read > 0) { printf("\n"); }
            }
            q->head = (q->head + n_read) & q->mask;
        }
        usleep(1000);
    }
    return NULL;
}

void *server_loop(void *ext) {
    ServerArgs *server_args = (ServerArgs *) ext;
    server_args->state = ServerStateIdle;
    while (*server_args->thread_run) {
        while (*server_args->arm == 0) {
            usleep(1000);
        }
        while (*server_args->run) {
            switch (server_args->state) {
            case ServerStateIdle:
                server_args->conn_fd = accept(server_args->sock_fd, &server_args->cli, sizeof(server_args->cli));
                if (server_args->conn_fd > 0) {
                    printf("accepted connection on port %d. connection = %d\n", server_args->port, server_args->conn_fd);
                    server_args->state = ServerStateConnected;
                }
                break;
            }
        }
    }
}

typedef struct ClientArgs {
    pthread_t *tid;
    int port;
    int sock_fd;
    int *arm;
    int *run;
    int *thread_run;
    char ip_addr[128];
} ClientArgs;

void *client_loop(void *ext) {

}

int open_socket(int server_mode, const char *ip_addr, int port) {
    if (server_mode) {
    struct sockaddr_in servaddr;
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd == -1) { return 0; }
    printf("socket %d successfully created\n", fd);
    bzero(&servaddr, sizeof (servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(port);
    if ((bind(fd, (struct sockaddr *) &servaddr, sizeof(servaddr))) != 0) {
        printf("socket failed to bind\n");
        return 0;
    }
    printf("socket bind success\n");
    set_blocking_mode(fd, 0);
    const int backlog = 5;
    if ((listen(fd, backlog)) != 0) {
        printf("server listen failed\n");
        return 0;
    }
    } else {
        struct sockaddr_in servaddr;
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd == -1) { return 0; }
        printf("socket %d successfully created\n", fd);
        bzero(&servaddr, sizeof (servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = inet_addr(ip_addr);
        servaddr.sin_port = htons(port);
    }
}

int main(int argc, char **argv)
{
#define QueueSize (512)
#define PoolSize (64)

    Pool tx_pool, rx_pool;
    initialize_pool(&tx_pool, PoolSize, QueueSize);
    initialize_pool(&rx_pool, PoolSize, QueueSize);
    Queue tx_queue, rx_queue;
    initialize_queue(&tx_queue, QueueSize);
    initialize_queue(&rx_queue, QueueSize);

    pthread_t socket_thread, device_rx_thread, device_tx_thread;
    int server_mode = 0;
    int fd_i = 0, fd_o = 0, sock_fd;
    ServerArgs *server_args = NULL;
    ClientArgs *client_args = NULL;
    RxLooperArgs *rx_looper_args = NULL;
    TxLooperArgs *tx_looper_args = NULL;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-debug") == 0) {
            debug = 1;
        } else if (strcmp(argv[i], "-verbose") == 0) {
            verbose = 1;
        } else if (strcmp(argv[i], "-n3") == 0) {
            n3 = 1;
        } else if (strcmp(argv[i], "-console") == 0) {
            fd_i = stdin;
            fd_o = stdout;
            rx_looper_args = (RxLooperArgs *) malloc(sizeof(RxLooperArgs));
            bzero(rx_looper_args, sizeof(RxLooperArgs));
            rx_looper_args->fd = stdin;
            rx_looper_args->rx_queue = &rx_queue;
            rx_looper_args->emit_pool = &rx_pool;
            tx_looper_args = (TxLooperArgs *) malloc(sizeof(TxLooperArgs));
            tx_looper_args->fd = stdout;
            tx_looper_args->
            bzero(tx_looper_args, sizeof(TxLooperArgs));
            pthread_create(&device_rx_thread, NULL, rx_looper, rx_looper_args);
            pthread_create(&device_tx_thread, NULL, tx_looper, tx_looper_args);
        } else if (strcmp(argv[i], "-uart") == 0) {
            const char *device_name = argv[++i];
            int baud_rate = atoi(argv[++i]);
            fd_o = fd_i = open_uart(device_name, baud_rate);
            rx_looper_args = (RxLooperArgs *) malloc(sizeof(RxLooperArgs));
            tx_looper_args = (TxLooperArgs *) malloc(sizeof(TxLooperArgs));
        } else if (strcmp(argv[i], "-server") == 0) {
            server_args = (ServerArgs *) malloc(sizeof(ServerArgs));
            server_args->port = atoi(argv[++i]);
            sock_fd = open_socket(1, NULL, server_args->port);
        } else if (strcmp(argv[i], "-client") == 0) {
            client_args = (ClientArgs *) malloc(sizeof(ClientArgs));
            snprintf(client_args->ip_addr, sizeof(client_args->ip_addr), argv[++i]);
            client_args->port = atoi(argv[++i]);
            sock_fd = open_socket(0, client_args->ip_addr, client_args->port);
        }
    }

    int thread_run = 1, arm = 0, run = 0;
    if (server_mode) {
        ServerArgs server_args;
        bzero(&server_args, sizeof(server_args));
        server_args.tid = &socket_thread;
        server_args.arm = &arm; /* make 0 to loop until armed. then set arm = 1 to drop into main loop */
        server_args.run = &run; /* run = 1 for main loop operation */
        server_args.thread_run = &thread_run; /* make 0 to quit */
        server_args.sock_fd = sock_fd;
        socket_thread = pthread_create(&socket_thread, NULL, server_loop, (void *) &server_args);
    } else {
        ClientArgs client_args;
        bzero(&client_args, sizeof(client_args));
        client_args.tid = &socket_thread;
        client_args.arm = &arm;
        client_args.run = &run;
        client_args.thread_run = &thread_run;
    }

    return 0;
}
