/*    queue.h

    Header file for queue implementation

    by: Steven Skiena
*/

/*
Copyright 2003 by Steven S. Skiena; all rights reserved.

Permission is granted for use in non-commerical applications
provided this copyright notice remains intact and unchanged.

This program appears in my book:

"Programming Challenges: The Programming Contest Training Manual"
by Steven Skiena and Miguel Revilla, Springer-Verlag, New York 2003.

See our website www.programming-challenges.com for additional information.

This book can be ordered from Amazon.com at

http://www.amazon.com/exec/obidos/ASIN/0387001638/thealgorithmrepo/

*/
#ifndef _QUEUE_H_
#define _QUEUE_H_

#define QUEUE_SIZE       1000

typedef struct
{
    USHORT first;                      /* position of first element */
    USHORT last;                       /* position of last element */
    USHORT count;                      /* number of queue elements */
    USHORT maxcount;                   /* max number of queue elements */
    //ULONG inQueueTimer;               /* time length in queue */
    HANDLE_Q q[1];                    /* body of queue */
} queue;


queue * define_new_queue(queue *pNewQueue, U16 queue_size);
U16 enqueue(queue *q, HANDLE_Q x);
HANDLE_Q dequeue(queue *q);
USHORT get_queue_cnt(queue *q);


#endif
