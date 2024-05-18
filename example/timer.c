#include "eloop.h"

static int i = 0;
static void timer_cb (struct eloop_base* base,struct eloop_timeout *t)
{
    fprintf(stderr,"timer count = %d priv = %d\n",i++,*((int*)(t->priv)));
    eloop_timeout_set(base,t,1000);

    if(i == 10)
        eloop_timeout_cancel(base,t);
}

int main(int argc, char const *argv[])
{
    /* code */
    struct eloop_base* base = eloop_init();

    //定时器
    int p = 10;
    struct eloop_timeout timer;
    memset(&timer,0,sizeof(timer));
    timer.cb = timer_cb;
    timer.priv = &p ;
    eloop_timeout_set(base,&timer,10);


    eloop_run(base);
    eloop_done(base);

    return 0;
}
