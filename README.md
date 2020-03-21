# libeloop
an easy event loop,support multi-thread,you can use in linux os

# 1.build with cmake
camke .

make

make install

# 2.how to use ?
you can leran from example

simple echo server code:
``` 
#include "eloop.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

static void read_cb (struct eloop_base* base,struct eloop_fd *efd, unsigned int events)
{
    fprintf(stderr,"have data\n");
    char buf[1024] = {0};
    int n = read(efd->fd,buf,sizeof(buf));
    if(n <= 0)
    {
        fprintf(stderr,"close connection\n");
        eloop_fd_delete(base,efd);
        close(efd->fd);
        free(efd);
        efd = NULL;
        return;
    }
    write(efd->fd,buf,n);
}

static void accept_cb (struct eloop_base* base,struct eloop_fd *efd, unsigned int events)
{
    fprintf(stderr,"new connection\n");

    struct sockaddr_in sin;
	unsigned int sl = sizeof(struct sockaddr_in);
	int cfd = accept(efd->fd, (struct sockaddr *) &sin, &sl);
	if (cfd < 0) 
    {
		fprintf(stderr, "Accept failed\n");
		return;
	}

    struct eloop_fd *pfd = (struct eloop_fd *)calloc(1,sizeof(*pfd));
    pfd->fd = cfd;
    pfd->cb = read_cb;
    eloop_fd_add(base,pfd,ELOOP_READ);

}

static int tcp_server_init(int port)
{
    int sock;

    struct sockaddr_in servaddr;

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
            return -1;

    
    const int one = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family      = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port        = htons(port);

    if(bind(sock, (struct sockaddr *)&servaddr, sizeof(servaddr)) != 0)
    {
        fprintf(stderr,"绑定端口失败\n");
        close(sock);
        return -3;
    }

    if(listen(sock, SOMAXCONN) != 0)
    {
        fprintf(stderr,"listen() 失败\n");
        close(sock);
        return -4;
    }
	
    return sock;
}

int main(int argc, char const *argv[])
{
    /* code */
    struct eloop_base* base = eloop_init();

    int sfd = tcp_server_init(5678);
    if(sfd < 0)
        return -1;
    
    struct eloop_fd efd;
    memset(&efd,0,sizeof(efd));

    efd.fd = sfd;
    efd.cb = accept_cb;

    eloop_fd_add(base,&efd,ELOOP_READ);

    eloop_run(base);
    eloop_done(base);

    return 0;
}

```




