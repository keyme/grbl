#include <gqueue.h>

void queue_enqueue(volatile void *q, const void *elt)
{
  volatile struct generic_queue *gq = q;

  memcpy((void*)gq->tail, elt, gq->item_size);

  if (gq->tail == gq->end) {
    gq->tail = gq->memory;
  } else {
    gq->tail = (void *)gq->tail + gq->item_size;
  }
}

void queue_dequeue(volatile void *q, void *elt)
{
  volatile struct generic_queue *gq = q;

  if (queue_get_len(q) == 0) {
    return;
  }

  memcpy(elt, (void*)gq->head, gq->item_size);

  if (gq->head == gq->end) {
    gq->head = gq->memory;
  } else {
    gq->head = (void*) gq->head + gq->item_size;
  }
}
