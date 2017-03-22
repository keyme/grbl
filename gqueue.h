#ifndef _QUEUE_H_
#define _QUEUE_H_

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>

struct generic_queue {
  volatile void *head;
  volatile void *tail;
  volatile void *end;
  unsigned int item_size;
  unsigned int max_capacity;
  volatile uint8_t memory[0];
};

#define DECLARE_QUEUE(name, element_type, max_size)	\
  static struct {					\
    struct generic_queue gq;				\
    element_type _elements[max_size + 1];              \
  } name;

static inline void queue_init(volatile void *q, int elt_size, int capacity)
{
  volatile struct generic_queue *gq = q;
  const size_t q_size = (sizeof(struct generic_queue) +
			 (elt_size * capacity));
  memset((void*)q, 0x00, q_size);
  gq->item_size = elt_size;
  gq->max_capacity = capacity;
  gq->head = gq->memory;
  gq->tail = gq->memory;
  gq->end  = gq->memory + (gq->max_capacity) * gq->item_size;
}

static inline bool queue_is_empty(volatile void *q)
{
  volatile struct generic_queue *gq = q;

  return (gq->head == gq->tail);
}

static inline unsigned int queue_get_len(volatile void *q)
{
  volatile struct generic_queue *gq = q;

  if (gq->tail == gq->head) {
    return 0;
  } else if (gq->tail > gq->head) {
    return (gq->tail - gq->head) / gq->item_size;
  } else {
    return gq->max_capacity - ((gq->head - gq->tail) / gq->item_size) + 1;
  }
}

static inline bool queue_is_full(volatile void *q)
{
  volatile struct generic_queue *gq = q;
  return (queue_get_len(q) >= gq->max_capacity);
}

void queue_enqueue(volatile void *q, const void *elt) __attribute__((nonnull));
void queue_dequeue(volatile void *q, void *elt) __attribute__((nonnull));

#endif
