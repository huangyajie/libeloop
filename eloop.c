/*
 * eloop - easy event loop implementation
 *
 * Copyright (C) 2019-2024 huang <https://github.com/huangyajie>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <fcntl.h>
#include <sys/types.h>
#include "eloop.h"

#ifdef USE_KQUEUE
#include <sys/event.h>
#endif
#ifdef USE_EPOLL
#include <sys/epoll.h>
#endif


#ifdef USE_EPOLL
//if does not define EPOLLRDHUP for Linux >= 2.6.17
#ifndef EPOLLRDHUP
#define EPOLLRDHUP 0x2000
#endif
#endif 


#define ELOOP_INIT_SIZE  1024
#define ELOOP_MAX_EVENTS 64

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif


struct eloop_fd_event 
{
	struct eloop_fd *fd;
	unsigned int events;
};


struct eloop_base
{
	int poll_fd;
	int cur_fd;
	int cur_nfds;
	int cur_trigger_nfds;
	bool eloop_cancelled;
	struct eloop_fd_event cur_fds[ELOOP_MAX_EVENTS];
	#ifdef USE_KQUEUE
	struct kevent events[ELOOP_MAX_EVENTS];
	#endif 
	#ifdef USE_EPOLL
	struct epoll_event events[ELOOP_MAX_EVENTS];
	#endif
	struct list_head timeouts;
};


//eloop init
struct eloop_base* eloop_init(void)
{
	struct eloop_base* base = calloc(1,sizeof(struct eloop_base));
	if(base == NULL)
		return NULL;

	#ifdef USE_KQUEUE
	struct timespec timeout = { 0, 0 };
	struct kevent ev = {};
	base->poll_fd = kqueue();
	if (base->poll_fd < 0)
	{
		free(base);
		base = NULL;
		return NULL;
	}
	EV_SET(&ev, SIGCHLD, EVFILT_SIGNAL, EV_ADD, 0, 0, 0);
	kevent(base->poll_fd, &ev, 1, NULL, 0, &timeout);
	#endif

    #ifdef USE_EPOLL
	base->poll_fd = epoll_create(ELOOP_INIT_SIZE);
	if (base->poll_fd < 0)
	{
		free(base);
		base = NULL;
		return NULL;
	}		
	fcntl(base->poll_fd, F_SETFD, fcntl(base->poll_fd, F_GETFD) | FD_CLOEXEC);
	#endif

	//init timer
	INIT_LIST_HEAD(&base->timeouts);
	
	return base;
}

#ifdef USE_KQUEUE
static uint16_t get_flags(unsigned int flags, unsigned int mask)
{
	uint16_t kflags = 0;

	if (!(flags & mask))
		return EV_DELETE;

	kflags = EV_ADD;
	if (flags & ELOOP_EDGE_TRIGGER)
		kflags |= EV_CLEAR;

	return kflags;
}
#endif

static int register_poll(struct eloop_base* base,struct eloop_fd *efd, unsigned int flags)
{
	if((base == NULL) || (efd == NULL))
		return ELOOP_FAIL;
	
	#ifdef USE_KQUEUE
	struct timespec timeout = { 0, 0 };
	struct kevent ev[2];
	int nev = 0;
	unsigned int fl = 0;
	unsigned int changed;
	uint16_t kflags;

	if (flags & ELOOP_EDGE_TRIGGER)
		flags |= ELOOP_EDGE_DEFER;
	else
		flags &= ~ELOOP_EDGE_DEFER;

	if (flags & ELOOP_EDGE_DEFER)
		flags &= ~ELOOP_EDGE_TRIGGER;

	changed = flags ^ efd->flags;
	if (changed & ELOOP_EDGE_TRIGGER)
		changed |= flags;

	if (!changed)
		return ELOOP_SUCCESS;

	if (changed & ELOOP_READ) 
	{
		kflags = get_flags(flags, ELOOP_READ);
		EV_SET(&ev[nev++], efd->fd, EVFILT_READ, kflags, 0, 0, efd);
	}

	if (changed & ELOOP_WRITE) 
	{
		kflags = get_flags(flags, ELOOP_WRITE);
		EV_SET(&ev[nev++], efd->fd, EVFILT_WRITE, kflags, 0, 0, efd);
	}

	if (!flags)
		fl |= EV_DELETE;

	return kevent(base->poll_fd, ev, nev, NULL, fl, &timeout); 
	#endif

	#ifdef USE_EPOLL
	struct epoll_event ev;
	int op = efd->registered ? EPOLL_CTL_MOD : EPOLL_CTL_ADD;

	memset(&ev, 0, sizeof(struct epoll_event));

	if (flags & ELOOP_READ)
		ev.events |= EPOLLIN | EPOLLRDHUP;

	if (flags & ELOOP_WRITE)
		ev.events |= EPOLLOUT;

	if (flags & ELOOP_EDGE_TRIGGER)
		ev.events |= EPOLLET;

	ev.data.fd = efd->fd;
	ev.data.ptr = efd;
	efd->flags = flags;

	return epoll_ctl(base->poll_fd, op, efd->fd, &ev);
	#endif
}

static int __eloop_fd_delete(struct eloop_base* base,struct eloop_fd *efd)
{
	if((base == NULL) || (efd == NULL))
		return ELOOP_FAIL;

	#ifdef USE_KQUEUE
	return register_poll(base,efd, 0);
	#endif

	#ifdef USE_EPOLL
	efd->flags = 0;
	return epoll_ctl(base->poll_fd, EPOLL_CTL_DEL, efd->fd, 0);
	#endif
}

//del fd event from eloop
int eloop_fd_delete(struct eloop_base* base,struct eloop_fd *efd)
{
	if((base == NULL) || (efd == NULL))
		return ELOOP_FAIL;
	
	int i = 0;

	for (i = 0; i < base->cur_nfds; i++) 
	{
		if (base->cur_fds[base->cur_fd + i].fd != efd)
			continue;

		base->cur_fds[base->cur_fd + i].fd = NULL;
	}

	if (!efd->registered)
		return ELOOP_SUCCESS;

	efd->registered = false;
	return __eloop_fd_delete(base,efd);
}

//add fd event to eloop
int eloop_fd_add(struct eloop_base* base,struct eloop_fd *efd, unsigned int flags)
{
	if((base == NULL) || (efd == NULL))
		return ELOOP_FAIL;

	unsigned int fl;

	if (!(flags & (ELOOP_READ | ELOOP_WRITE)))
		return eloop_fd_delete(base,efd);

	if (!efd->registered && !(flags & ELOOP_BLOCKING)) 
	{
		fl = fcntl(efd->fd, F_GETFL, 0);
		fl |= O_NONBLOCK;
		fcntl(efd->fd, F_SETFL, fl);
	}

	int ret = register_poll(base,efd, flags);
	if (ret < 0)
		return ELOOP_FAIL;

	efd->registered = true;
	efd->eof = false;
	efd->error = false;

    return ELOOP_SUCCESS;
}


static int eloop_fetch_events(struct eloop_base* base,int timeout)
{
	if(base == NULL)
		return ELOOP_FAIL;
	int n, nfds;

	#ifdef USE_KQUEUE
	struct timespec ts;
	if (timeout >= 0) {
		ts.tv_sec = timeout / 1000;
		ts.tv_nsec = (timeout % 1000) * 1000000;
	}

	nfds = kevent(base->poll_fd, NULL, 0, base->events, ARRAY_SIZE(base->events), timeout >= 0 ? &ts : NULL);
	for (n = 0; n < nfds; n++) 
	{
		struct eloop_fd_event *cur = &base->cur_fds[n];
		struct eloop_fd *u = base->events[n].udata;
		unsigned int ev = 0;

		cur->fd = u;
		if (!u)
			continue;

		if (base->events[n].flags & EV_ERROR) 
		{
			u->error = true;
			if (!(u->flags & ELOOP_ERROR_CB))
				eloop_fd_delete(base,u);
		}

		if(base->events[n].filter == EVFILT_READ)
			ev |= ELOOP_READ;
		else if (base->events[n].filter == EVFILT_WRITE)
			ev |= ELOOP_WRITE;

		if (base->events[n].flags & EV_EOF)
			u->eof = true;
		else if (!ev)
			cur->fd = NULL;

		cur->events = ev;
		if (u->flags & ELOOP_EDGE_DEFER) 
		{
			u->flags &= ~ELOOP_EDGE_DEFER;
			u->flags |= ELOOP_EDGE_TRIGGER;
			register_poll(base,u, u->flags);
		}
	}
	#endif 

	#ifdef USE_EPOLL
	nfds = epoll_wait(base->poll_fd, base->events, ARRAY_SIZE(base->events), timeout);
	for (n = 0; n < nfds; n++) 
	{
		struct eloop_fd_event *cur = &base->cur_fds[n];
		struct eloop_fd *u = base->events[n].data.ptr;
		unsigned int ev = 0;

		cur->fd = u;
		if (!u)
			continue;

		if (base->events[n].events & (EPOLLERR|EPOLLHUP)) 
		{
			u->error = true;
			if (!(u->flags & ELOOP_ERROR_CB))
				eloop_fd_delete(base,u);
		}

		if(!(base->events[n].events & (EPOLLRDHUP|EPOLLIN|EPOLLOUT|EPOLLERR|EPOLLHUP))) 
		{
			cur->fd = NULL;
			continue;
		}

		if(base->events[n].events & EPOLLRDHUP)
			u->eof = true;

		if(base->events[n].events & EPOLLIN)
			ev |= ELOOP_READ;

		if(base->events[n].events & EPOLLOUT)
			ev |= ELOOP_WRITE;

		cur->events = ev;
	}
	#endif 

	return nfds;
}



static void eloop_run_events(struct eloop_base* base,int timeout)
{
	if(base == NULL)
		return;

	struct eloop_fd_event *cur = NULL;
	struct eloop_fd *fd = NULL;

	if (!base->cur_nfds) 
	{
		base->cur_fd = 0;
		base->cur_nfds = eloop_fetch_events(base,timeout);
		if (base->cur_nfds < 0)
			base->cur_nfds = 0;

		base->cur_trigger_nfds = base->cur_nfds;
	}

	while (base->cur_nfds > 0) 
	{
		unsigned int events;

		cur = &base->cur_fds[base->cur_fd++];
		base->cur_nfds--;

		fd = cur->fd;
		events = cur->events;
		if (!fd)
			continue;

		if (!fd->cb)
			continue;

		fd->cb(base,fd, events);

	}
}



static int tv_diff(struct timeval *t1, struct timeval *t2)
{
	return
		(t1->tv_sec - t2->tv_sec) * 1000 +
		(t1->tv_usec - t2->tv_usec) / 1000;
}

//timer add
static int eloop_timeout_add(struct eloop_base* base,struct eloop_timeout *timeout)
{
	if((base == NULL) || (timeout == NULL) )
		return ELOOP_FAIL;

	struct eloop_timeout *tmp;
	struct list_head *h = &base->timeouts;

	if (timeout->pending)
		return ELOOP_FAIL;

	list_for_each_entry(tmp, &base->timeouts, list) 
	{
		if (tv_diff(&tmp->time, &timeout->time) > 0) 
		{
			h = &tmp->list;
			break;
		}
	}

	list_add_tail(&timeout->list, h);
	timeout->pending = true;

	return ELOOP_SUCCESS;
}

static void eloop_gettime(struct timeval *tv)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	tv->tv_sec = ts.tv_sec;
	tv->tv_usec = ts.tv_nsec / 1000;
}

//timer set
int eloop_timeout_set(struct eloop_base* base,struct eloop_timeout *timeout, int msecs)
{
	if((base == NULL) || (timeout == NULL) )
		return ELOOP_FAIL;

	struct timeval *time = &timeout->time;

	if (timeout->pending)
		eloop_timeout_cancel(base,timeout);

	eloop_gettime(&timeout->time);

	time->tv_sec += msecs / 1000;
	time->tv_usec += (msecs % 1000) * 1000;

	if (time->tv_usec > 1000000) {
		time->tv_sec++;
		time->tv_usec %= 1000000;
	}

	return eloop_timeout_add(base,timeout);
}

//cancel timer
int eloop_timeout_cancel(struct eloop_base* base,struct eloop_timeout *timeout)
{
	if((base == NULL) || (timeout == NULL) )
		return ELOOP_FAIL;

	if (!timeout->pending)
		return ELOOP_FAIL;

	list_del(&timeout->list);
	timeout->pending = false;

	return ELOOP_SUCCESS;
}

//get timer ramaining time (ms)
int eloop_timeout_remaining(struct eloop_base* base,struct eloop_timeout *timeout)
{
	if((base == NULL) || (timeout == NULL) )
		return ELOOP_FAIL;

	struct timeval now;

	if (!timeout->pending)
		return -1;

	eloop_gettime(&now);

	return tv_diff(&timeout->time, &now);
}


static int eloop_get_next_timeout(struct eloop_base* base,struct timeval *tv)
{
	if((base == NULL) || (tv == NULL) )
		return ELOOP_FAIL;

	struct eloop_timeout *timeout;
	int diff;

	if (list_empty(&base->timeouts))
		return ELOOP_FAIL;

	timeout = list_first_entry(&base->timeouts, struct eloop_timeout, list);
	diff = tv_diff(&timeout->time, tv);
	if (diff < 0)
		return ELOOP_SUCCESS;

	return diff;
}

static void eloop_process_timeouts(struct eloop_base* base,struct timeval *tv)
{
	if((base == NULL) || (tv == NULL) )
		return;

	struct eloop_timeout *t;

	while (!list_empty(&base->timeouts)) 
	{
		t = list_first_entry(&base->timeouts, struct eloop_timeout, list);

		if (tv_diff(&t->time, tv) > 0)
			break;

		eloop_timeout_cancel(base,t);
		if (t->cb)
			t->cb(base,t);
	}
}

static void eloop_clear_timeouts(struct eloop_base* base)
{
	if( base == NULL )
		return ;
	struct eloop_timeout *t, *tmp;

	list_for_each_entry_safe(t, tmp, &base->timeouts, list)
		eloop_timeout_cancel(base,t);
}



// enter event loop
int eloop_run(struct eloop_base* base)
{
	if(base == NULL)
		return ELOOP_FAIL;


	struct timeval tv;
	base->eloop_cancelled = false;
	while(!base->eloop_cancelled)
	{
		eloop_gettime(&tv);
		eloop_process_timeouts(base,&tv);
		if (base->eloop_cancelled)
			break;
		eloop_gettime(&tv);
		eloop_run_events(base,eloop_get_next_timeout(base,&tv));
	}

	return ELOOP_SUCCESS;
}

// cancel event loop
int eloop_end(struct eloop_base* base)
{
	if(base == NULL)
		return ELOOP_FAIL;

	base->eloop_cancelled = true;

	return ELOOP_SUCCESS;
}


// event loop is done
int eloop_done(struct eloop_base* base)
{
	if(base == NULL)
		return ELOOP_FAIL;

	if (base->poll_fd < 0)
		return ELOOP_FAIL;

	close(base->poll_fd);
	base->poll_fd = -1;

	eloop_clear_timeouts(base);

	if(base != NULL)
	{
		free(base);
		base = NULL;
	}
	
	return ELOOP_SUCCESS;
}

int eloop_get_trigger_events(struct eloop_base* base,struct poll_fd* pfd, unsigned int out_sz)
{
	if(base == NULL)
		return ELOOP_FAIL;

	int i = 0;
	for(i = 0;i < base->cur_trigger_nfds;i++)
	{
		if(i > out_sz - 1)
		{
			break;
		}
		pfd[i].fd = base->cur_fds[i].fd->fd;
		pfd[i].events = base->cur_fds[i].events;
	}

	return i;
}