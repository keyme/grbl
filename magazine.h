/*
  Not part of Grbl. KeyMe specific.
*/

#ifndef magazine_h
#define magazine_h

#define MAX_EDGE_EVENTS 2

// Magazine allignment pin initialization routine
void magazine_init();

#define magazine_get_state() (!(MAGAZINE_ALIGNMENT_PIN & MAGAZINE_ALIGNMENT_MASK))

// Monitors the gap in units between mags and throws an alarm if the gap is larger than a
// specified threshold.
void magazine_gap_monitor();

void magazine_report_edge_events(void);

#endif
