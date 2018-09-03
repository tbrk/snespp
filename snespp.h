/*
 * Copyright (c) 2007 Timothy Bourke.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Id: snespp.h 31 2007-04-16 08:33:29Z root $
 */

#ifndef __SNESPP_H
#define __SNESPP_H

#include <dev/ppbus/lpt.h>

#define SNESPP_NAME	"snespp"

/** Controller interface **/

				    	/* PIN */
#define SNESD_CLOCK	0x01		/*  2   data0   */
#define SNESD_LATCH	0x02		/*  3   data1   */
#define SNESD_POWER	0xfc		/*  4-9 data2-7 */

#define SNESPP_PAD0	LPS_NACK	/*  10  pad 0   */
#define SNESPP_PAD1	LPS_OUT		/*  12  pad 1   */
#define SNESPP_PAD2	LPS_NBSY	/*  11  pad 2   */
#define SNESPP_PAD3	LPS_SEL		/*  13  pad 3   */
#define SNESPP_PAD4	LPS_NERR	/*  15  pad 4   */

#define SNESPP_NUM_BUTTONS	12
#define SNESPP_B		0
#define SNESPP_Y		1
#define SNESPP_SELECT		2
#define SNESPP_START		3
#define SNESPP_UP		4
#define SNESPP_DOWN		5
#define SNESPP_LEFT		6
#define SNESPP_RIGHT		7
#define SNESPP_A		8
#define SNESPP_X		9
#define SNESPP_L		10
#define SNESPP_R		11

#define SNESPP_JOY_MIN		-2000
#define SNESPP_JOY_MID		0
#define SNESPP_JOY_MAX		2000

/** Polling **/
#define SNESPP_NUM_CONTROLLERS	2
#define SNESPP_EVENTQ_SIZE	(30 * SNESPP_NUM_CONTROLLERS)
#define SNESPP_POLL_HZ		60
#define SNESPP_TASK_FREQ	6

/** Functions and structures in common **/
struct snespp_softc {
	struct cdev *		dev;
	device_t		device;

	short			state;
#define HAVEBUS			0x0001		/* driver owns the bus */
#define TASKPENDING		0x0002		/* rd_task pending */ 	
#define POLLINGREQUESTED	0x0004		/* poll the controller */
#define SNESDOPEN		0x0008		/* snes0 device open */

	short			curbuffer;
	char			buffer[2][SNESPP_NUM_BUTTONS];

	int			eventq_active;
	struct			mtx eventq_lock;

	/* Circular buffer. Write at head, read from tail (must hold
	 * eventq_lock). Empty when eq_head == eq_tail. We accept data
	 * loss if the head overtakes the tail; the most recent events
	 * are of most importance. */
	unsigned int		eventq[SNESPP_EVENTQ_SIZE];
	int			eq_head;
	int			eq_tail;

	int			run_rd_task;
	struct task		rd_task;
};

int  snespp_queue_getc(struct snespp_softc *sc);
int  snespp_queue_empty(struct snespp_softc *sc);
void snespp_queue_clear(struct snespp_softc *sc);
void snespp_clearpending(struct snespp_softc *sc);
void snespp_requestpolling(struct snespp_softc *sc, int poll);

/** Keyboard interface **/
int snesk_create(struct snespp_softc *);
int snesk_destroy(struct snespp_softc *);

#endif
