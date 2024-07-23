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

#ifndef _ELOOP_H_
#define _ELOOP_H_

#ifdef __cpluscplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <errno.h>
#include <signal.h>
#include "list.h"

#if defined(__APPLE__) || defined(__FreeBSD__)
#define USE_KQUEUE
#else
#define USE_EPOLL
#endif



#define ELOOP_READ		(1 << 0)
#define ELOOP_WRITE		(1 << 1)
#define ELOOP_EDGE_TRIGGER	(1 << 2)
#define ELOOP_BLOCKING		(1 << 3)

/* internal flags */
#ifdef USE_KQUEUE
#define ELOOP_EDGE_DEFER	(1 << 5)
#endif
/* internal flags */
#define ELOOP_ERROR_CB		(1 << 6)

enum
{
	ELOOP_FAIL = -1,
	ELOOP_SUCCESS = 0
};

struct eloop_timeout;
struct eloop_fd;
struct eloop_base;

typedef void (*eloop_fd_handler)(struct eloop_base* base,struct eloop_fd *efd, unsigned int events);
typedef void (*eloop_timeout_handler)(struct eloop_base* base,struct eloop_timeout *t);

//fd event structure
struct eloop_fd
{
	eloop_fd_handler cb;  //callback  function
	int fd; //fd
	bool eof;  //caller does not care
	bool error; //fd is occur error or not ,caller does not care
	bool registered; //fd is registered or not,caller does not care
	uint8_t flags; //event type ,such as ELOOP_READ
	void* priv; //caller privite data
};

//timer event structure
struct eloop_timeout
{
	struct list_head list; //caller does not care
	bool pending; //timer is pending or not ,caller does not care
	eloop_timeout_handler cb;  //timeout callback function
	struct timeval time; //caller does not care
	void* priv; //caller privite data
};

struct poll_fd
{
    int fd;
    unsigned int events;
};



//eloop init
struct eloop_base* eloop_init(void);

//add fd event to eloop
int eloop_fd_add(struct eloop_base* base,struct eloop_fd *efd, unsigned int flags);

//del fd event from eloop
int eloop_fd_delete(struct eloop_base* base,struct eloop_fd *efd);

// enter event loop
int eloop_run(struct eloop_base* base);

// cancel event loop
int eloop_end(struct eloop_base* base);

// event loop is done
int eloop_done(struct eloop_base* base);

//timer set
int eloop_timeout_set(struct eloop_base* base,struct eloop_timeout *timeout, int msecs);


//cancel timer
int eloop_timeout_cancel(struct eloop_base* base,struct eloop_timeout *timeout);

//get timer ramaining time (ms)
int eloop_timeout_remaining(struct eloop_base* base,struct eloop_timeout *timeout);

//get trigger evernts  
int eloop_get_trigger_events(struct eloop_base* base,struct poll_fd* pfd, unsigned int out_sz);

#ifdef __cpluscplus
}
#endif

#endif
