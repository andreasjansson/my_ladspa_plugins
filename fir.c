/* 
 * fir.c - A simple one-term finite impulse response filter
 * Copyright (C) 2011  Andreas Jansson <andreas@jansson.me.uk>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at
 * your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

/**
 * This simple FIR filter adds dips in the frequency response at odd
 * multiples of some initial frequency. It comes with both a mono
 * and a stereo version. In the stereo version you can have separate
 * configurations for the two channels.
 */

#include <stdlib.h>
#include <string.h>

#include "ladspa.h"

#define MIN_FREQ 1

// The port numbers for the plugin
#define FREQ_CONTROL_L 0
#define WET_CONTROL_L  1
#define INPUT_L        2
#define OUTPUT_L       3

#define FREQ_CONTROL_R 4
#define WET_CONTROL_R  5
#define INPUT_R        6
#define OUTPUT_R       7

LADSPA_Descriptor *mono_descriptor = NULL;
LADSPA_Descriptor *stereo_descriptor = NULL;

/**
 * Structure to hold connections and state.
 */
typedef struct {
  LADSPA_Data *freq_control_value_l;
  LADSPA_Data *wet_control_value_l;
  LADSPA_Data *freq_control_value_r;
  LADSPA_Data *wet_control_value_r;

  // l = mono
  LADSPA_Data *input_buffer_l;
  LADSPA_Data *output_buffer_l;

  // stereo
  LADSPA_Data *input_buffer_r;
  LADSPA_Data *output_buffer_r;

  // state
  unsigned long sample_rate;

  // The history array is a circular buffer used to
  // keep track of old samples.
  LADSPA_Data *history_l;
  LADSPA_Data *history_r;
  unsigned long history_position;
  unsigned long history_length;
} filter_type;


inline unsigned long get_sample_shift(float freq, unsigned long sample_rate)
{
  return (unsigned long)((1 / (2 * freq)) * sample_rate);
}

/**
 * Construct a new plugin instance.
 */
LADSPA_Handle instantiate_filter(const LADSPA_Descriptor *descriptor,
                                 unsigned long sample_rate)
{
  filter_type *filter = malloc(sizeof(filter_type));
  filter->sample_rate = sample_rate;

  return filter;
}

void activate_filter(LADSPA_Handle instance, int stereo)
{
  filter_type *filter = (filter_type *)instance;
  filter->history_position = 0;
  filter->history_length = get_sample_shift(MIN_FREQ, filter->sample_rate);
  filter->history_l = calloc(filter->history_length,
                             sizeof(LADSPA_Descriptor));
  if(stereo)
    filter->history_r = calloc(filter->history_length,
                               sizeof(LADSPA_Descriptor));
  else
    filter->history_r = NULL;
}

void activate_mono_filter(LADSPA_Handle instance)
{
  activate_filter(instance, 0);
}

void activate_stereo_filter(LADSPA_Handle instance)
{
  activate_filter(instance, 1);
}

/**
 * Connect a port to a data location. 
*/
void connect_port_to_filter(LADSPA_Handle instance,
                            unsigned long port,
                            LADSPA_Data *data_location)
{
  filter_type *filter;

  filter = (filter_type *)instance;
  switch(port) {
  case FREQ_CONTROL_L:
    filter->freq_control_value_l = data_location;
    break;
  case WET_CONTROL_L:
    filter->wet_control_value_l = data_location;
    break;
  case FREQ_CONTROL_R:
    filter->freq_control_value_r = data_location;
    break;
  case WET_CONTROL_R:
    filter->wet_control_value_r = data_location;
    break;
  case INPUT_L:
    filter->input_buffer_l = data_location;
    break;
  case OUTPUT_L:
    filter->output_buffer_l = data_location;
    break;
  case INPUT_R:
    filter->input_buffer_r = data_location;
    break;
  case OUTPUT_R:
    filter->output_buffer_r = data_location;
    break;
  }
}

/**
 * This is where the action happens.
 */
void run_filter(LADSPA_Handle instance, unsigned long sample_count,
                int stereo)
{
  LADSPA_Data *input_l;
  LADSPA_Data *input_r = NULL;
  LADSPA_Data *output_l;
  LADSPA_Data *output_r = NULL;
  filter_type *filter;
  unsigned long sample_shift_l;
  unsigned long sample_shift_r = 0;

  filter = (filter_type *)instance;
  input_l = filter->input_buffer_l;
  output_l = filter->output_buffer_l;

  if(stereo) {
    input_r = filter->input_buffer_r;
    output_r = filter->output_buffer_r;
  }

  // get the current sample shift as a function of the frequency
  // control value set by the user.
  sample_shift_l = get_sample_shift(*filter->freq_control_value_l,
                                    filter->sample_rate);
  if(stereo)
    sample_shift_r = get_sample_shift(*filter->freq_control_value_r,
                                      filter->sample_rate);

  while(sample_count -- > 0) {

    // add the current sample <sample_shift> steps ahead in the history
    // buffer. this is the way we maintain the delay.
    *(filter->history_l + ((sample_shift_l + filter->history_position) %
                           filter->history_length)) = *input_l;

    if(stereo)
      *(filter->history_r + ((sample_shift_r + filter->history_position) %
         filter->history_length)) = *input_r;
    
    *output_l = *input_l * (1 - *filter->wet_control_value_l / 2) +
      *(filter->history_l + filter->history_position) *
      *filter->wet_control_value_l / 2;

    if(stereo)
      *output_r = *input_r * (1 - *filter->wet_control_value_r / 2) +
        *(filter->history_r + filter->history_position) *
        *filter->wet_control_value_r / 2;

    filter->history_position = (filter->history_position + 1) %
      filter->history_length;

    input_l ++;
    output_l ++;

    if(stereo) {
      input_r ++;
      output_r ++;
    }
  }
}

void run_mono_filter(LADSPA_Handle instance, unsigned long sample_count)
{
  run_filter(instance, sample_count, 0);
}

void run_stereo_filter(LADSPA_Handle instance, unsigned long sample_count)
{
  run_filter(instance, sample_count, 1);
}

void deactivate_filter(LADSPA_Handle instance)
{
  filter_type *filter = (filter_type *)instance;
  free(filter->history_l);
  if(filter->history_r)
    free(filter->history_r);
}

void cleanup_filter(LADSPA_Handle instance)
{
  free(instance);
}

/**
 * The constructor function is called automatically
 * when the plugin library is first loaded.
 * This is where we build the descriptors that the host
 * will be using.
 */
void __attribute__ ((constructor)) init(void)
{
  char **port_names;
  LADSPA_PortDescriptor *port_descriptors;
  LADSPA_PortRangeHint *port_range_hints;

  mono_descriptor = (LADSPA_Descriptor *)malloc(sizeof(LADSPA_Descriptor));
  stereo_descriptor = (LADSPA_Descriptor *)malloc(sizeof(LADSPA_Descriptor));

  if(mono_descriptor) {
    mono_descriptor->UniqueID = 0x00654321;
    mono_descriptor->Label = strdup("fir_mono");
    mono_descriptor->Properties = LADSPA_PROPERTY_HARD_RT_CAPABLE;
    mono_descriptor->Name = strdup("One-term FIR filter (mono)");
    mono_descriptor->Maker = strdup("Andreas Jansson");
    mono_descriptor->Copyright = strdup("GPL-3.0");
    mono_descriptor->PortCount = 4;

    port_descriptors
      = (LADSPA_PortDescriptor *)calloc(4, sizeof(LADSPA_PortDescriptor));
    mono_descriptor->PortDescriptors
      = (const LADSPA_PortDescriptor *)port_descriptors;
    port_descriptors[WET_CONTROL_L] = LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
    port_descriptors[FREQ_CONTROL_L] = LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
    port_descriptors[INPUT_L] = LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO;
    port_descriptors[OUTPUT_L] = LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO;

    port_names = (char **)calloc(4, sizeof(char *));
    mono_descriptor->PortNames = (const char **)port_names;
    port_names[WET_CONTROL_L] = strdup("Dry/Wet");
    port_names[FREQ_CONTROL_L] = strdup("First frequency");
    port_names[INPUT_L] = strdup("Input");
    port_names[OUTPUT_L] = strdup("Output");

    port_range_hints =
      ((LADSPA_PortRangeHint *)calloc(4, sizeof(LADSPA_PortRangeHint)));
    mono_descriptor->PortRangeHints =
      (const LADSPA_PortRangeHint *)port_range_hints;
    port_range_hints[WET_CONTROL_L].HintDescriptor =
      (LADSPA_HINT_BOUNDED_BELOW
       | LADSPA_HINT_BOUNDED_ABOVE
       | LADSPA_HINT_DEFAULT_0);
    port_range_hints[WET_CONTROL_L].LowerBound = 0;
    port_range_hints[WET_CONTROL_L].UpperBound = 1;
    port_range_hints[FREQ_CONTROL_L].HintDescriptor =
      (LADSPA_HINT_BOUNDED_BELOW
       | LADSPA_HINT_BOUNDED_ABOVE
       | LADSPA_HINT_LOGARITHMIC
       | LADSPA_HINT_INTEGER
       | LADSPA_HINT_DEFAULT_LOW);
    port_range_hints[FREQ_CONTROL_L].LowerBound = 20;
    port_range_hints[FREQ_CONTROL_L].UpperBound = 20000;
    port_range_hints[INPUT_L].HintDescriptor = 0;
    port_range_hints[OUTPUT_L].HintDescriptor = 0;

    mono_descriptor->instantiate = instantiate_filter;
    mono_descriptor->connect_port = connect_port_to_filter;
    mono_descriptor->activate = activate_mono_filter;
    mono_descriptor->run = run_mono_filter;
    mono_descriptor->run_adding = NULL;
    mono_descriptor->set_run_adding_gain = NULL;
    mono_descriptor->deactivate = deactivate_filter;
    mono_descriptor->cleanup = cleanup_filter;
  }

  if(stereo_descriptor) {
    stereo_descriptor->UniqueID = 0x00654322;
    stereo_descriptor->Label = strdup("fir_stereo");
    stereo_descriptor->Properties = LADSPA_PROPERTY_HARD_RT_CAPABLE;
    stereo_descriptor->Name = strdup("One-term FIR filter (stereo)");
    stereo_descriptor->Maker = strdup("Andreas Jansson");
    stereo_descriptor->Copyright = strdup("GPL-3.0");
    stereo_descriptor->PortCount = 8;

    port_descriptors
      = (LADSPA_PortDescriptor *)calloc(8, sizeof(LADSPA_PortDescriptor));
    stereo_descriptor->PortDescriptors
      = (const LADSPA_PortDescriptor *)port_descriptors;
    port_descriptors[WET_CONTROL_L] = LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
    port_descriptors[FREQ_CONTROL_L] = LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
    port_descriptors[WET_CONTROL_R] = LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
    port_descriptors[FREQ_CONTROL_R] = LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
    port_descriptors[INPUT_L] = LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO;
    port_descriptors[OUTPUT_L] = LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO;
    port_descriptors[INPUT_R] = LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO;
    port_descriptors[OUTPUT_R] = LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO;

    port_names = (char **)calloc(8, sizeof(char *));
    stereo_descriptor->PortNames = (const char **)port_names;
    port_names[WET_CONTROL_L] = strdup("Dry/Wet Left");
    port_names[FREQ_CONTROL_L] = strdup("First frequency Left");
    port_names[WET_CONTROL_R] = strdup("Dry/Wet Right");
    port_names[FREQ_CONTROL_R] = strdup("First frequency Right");
    port_names[INPUT_L] = strdup("Input Left");
    port_names[OUTPUT_L] = strdup("Output Left");
    port_names[INPUT_R] = strdup("Input Right");
    port_names[OUTPUT_R] = strdup("Output Right");

    port_range_hints =
      ((LADSPA_PortRangeHint *)calloc(8, sizeof(LADSPA_PortRangeHint)));
    stereo_descriptor->PortRangeHints =
      (const LADSPA_PortRangeHint *)port_range_hints;
    port_range_hints[WET_CONTROL_L].HintDescriptor =
      (LADSPA_HINT_BOUNDED_BELOW
       | LADSPA_HINT_BOUNDED_ABOVE
       | LADSPA_HINT_DEFAULT_0);
    port_range_hints[WET_CONTROL_L].LowerBound = 0;
    port_range_hints[WET_CONTROL_L].UpperBound = 1;
    port_range_hints[FREQ_CONTROL_L].HintDescriptor =
      (LADSPA_HINT_BOUNDED_BELOW
       | LADSPA_HINT_BOUNDED_ABOVE
       | LADSPA_HINT_LOGARITHMIC
       | LADSPA_HINT_INTEGER
       | LADSPA_HINT_DEFAULT_LOW);
    port_range_hints[FREQ_CONTROL_L].LowerBound = 20;
    port_range_hints[FREQ_CONTROL_L].UpperBound = 20000;
    port_range_hints[WET_CONTROL_R].HintDescriptor =
      (LADSPA_HINT_BOUNDED_BELOW
       | LADSPA_HINT_BOUNDED_ABOVE
       | LADSPA_HINT_DEFAULT_0);
    port_range_hints[WET_CONTROL_R].LowerBound = 0;
    port_range_hints[WET_CONTROL_R].UpperBound = 1;
    port_range_hints[FREQ_CONTROL_R].HintDescriptor =
      (LADSPA_HINT_BOUNDED_BELOW
       | LADSPA_HINT_BOUNDED_ABOVE
       | LADSPA_HINT_LOGARITHMIC
       | LADSPA_HINT_INTEGER
       | LADSPA_HINT_DEFAULT_LOW);
    port_range_hints[FREQ_CONTROL_R].LowerBound = 20;
    port_range_hints[FREQ_CONTROL_R].UpperBound = 20000;
    port_range_hints[INPUT_L].HintDescriptor = 0;
    port_range_hints[OUTPUT_L].HintDescriptor = 0;
    port_range_hints[INPUT_R].HintDescriptor = 0;
    port_range_hints[OUTPUT_R].HintDescriptor = 0;

    stereo_descriptor->instantiate = instantiate_filter;
    stereo_descriptor->connect_port = connect_port_to_filter;
    stereo_descriptor->activate = activate_stereo_filter;
    stereo_descriptor->run = run_stereo_filter;
    stereo_descriptor->run_adding = NULL;
    stereo_descriptor->set_run_adding_gain = NULL;
    stereo_descriptor->deactivate = deactivate_filter;
    stereo_descriptor->cleanup = cleanup_filter;
  }

}

void delete_descriptor(LADSPA_Descriptor *descriptor)
{
  unsigned long i;
  if(descriptor) {
    free((char *)descriptor->Label);
    free((char *)descriptor->Name);
    free((char *)descriptor->Maker);
    free((char *)descriptor->Copyright);
    free((LADSPA_PortDescriptor *)descriptor->PortDescriptors);

    for(i = 0; i < descriptor->PortCount; i ++)
      free((char *)(descriptor->PortNames[i]));

    free((char **)descriptor->PortNames);
    free((LADSPA_PortRangeHint *)descriptor->PortRangeHints);

    free(descriptor);
  }
}

/**
 * The destructor function is called automatically when
 * the library is unloaded.
 */
void __attribute__ ((destructor)) fini(void)
{
  delete_descriptor(mono_descriptor);
  delete_descriptor(stereo_descriptor);
}

/* Return a descriptor of the requested plugin type. There are two
   plugin types available in this library (mono and stereo). */
const LADSPA_Descriptor *ladspa_descriptor(unsigned long index)
{
  /* Return the requested descriptor or null if the index is out of
     range. */
  switch (index) {
  case 0:
    return mono_descriptor;
  case 1:
    return stereo_descriptor;
  default:
    return NULL;
  }
}

