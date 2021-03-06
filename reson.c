/* 
 * reson.c - A simple two-pole resonance filter
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
 * A simple 2-pole reson filter. Reson filters attenuate frequencies
 * below and above a resonant frequency. User-definable parameters
 * are frequency and bandwidth.
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "ladspa.h"

// The port numbers for the plugin
#define FREQ_CONTROL_L 0
#define BW_CONTROL_L   1
#define INPUT_L        2
#define OUTPUT_L       3

#define FREQ_CONTROL_R 4
#define BW_CONTROL_R   5
#define INPUT_R        6
#define OUTPUT_R       7

LADSPA_Descriptor *mono_descriptor = NULL;
LADSPA_Descriptor *stereo_descriptor = NULL;

/**
 * Structure to hold connections and state.
 */
typedef struct {
  LADSPA_Data *freq_control_value_l;
  LADSPA_Data *bw_control_value_l;
  LADSPA_Data *freq_control_value_r;
  LADSPA_Data *bw_control_value_r;

  // l = mono
  LADSPA_Data *input_buffer_l;
  LADSPA_Data *output_buffer_l;

  // stereo
  LADSPA_Data *input_buffer_r;
  LADSPA_Data *output_buffer_r;

  // state
  unsigned long sample_rate;

  // keep the two most recent samples
  LADSPA_Data *history_l;
  LADSPA_Data *history_r;
} filter_type;


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
  filter->history_l = calloc(2, sizeof(LADSPA_Descriptor));
  if(stereo)
    filter->history_r = calloc(2, sizeof(LADSPA_Descriptor));
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
  case BW_CONTROL_L:
    filter->bw_control_value_l = data_location;
    break;
  case FREQ_CONTROL_R:
    filter->freq_control_value_r = data_location;
    break;
  case BW_CONTROL_R:
    filter->bw_control_value_r = data_location;
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

inline void filter_channel(LADSPA_Data *input, LADSPA_Data *output,
                           float freq, float bw, LADSPA_Data *history,
                           unsigned long sample_count,
                           unsigned long sample_rate)
{
  float pole_radius;
  float pole_angle;
  float gain_factor;

  pole_radius = 1 - M_PI * bw / sample_rate;
  pole_angle = acos(((2 * pole_radius) / (1 + pow(pole_radius, 2))) *
                      cos(2 * M_PI * freq / sample_rate));
  
  gain_factor = (1 - pow(pole_radius, 2)) * sin(pole_angle);
  
  while(sample_count -- > 0) {
    *output = gain_factor * *input + 2 * pole_radius *
      cos(pole_angle) * history[0] - pow(pole_radius, 2) * history[1];

    history[1] = history[0];
    history[0] = *output;
    output ++;
    input ++;
  }
}

/**
 * This is where the action happens.
 */
inline void run_filter(LADSPA_Handle instance, unsigned long sample_count,
                       int stereo)
{
  filter_type *filter;
  filter = (filter_type *)instance;

  filter_channel(filter->input_buffer_l, filter->output_buffer_l,
                 *(float *)filter->freq_control_value_l,
                 *(float *)filter->bw_control_value_l,
                 filter->history_l, sample_count, filter->sample_rate);

  if(stereo)
    filter_channel(filter->input_buffer_r, filter->output_buffer_r,
                   *(float *)filter->freq_control_value_r,
                   *(float *)filter->bw_control_value_r,
                   filter->history_r, sample_count, filter->sample_rate);
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
    mono_descriptor->UniqueID = 0x00654325;
    mono_descriptor->Label = strdup("reson_mono");
    mono_descriptor->Properties = LADSPA_PROPERTY_HARD_RT_CAPABLE;
    mono_descriptor->Name = strdup("Two-pole reson filter (mono)");
    mono_descriptor->Maker = strdup("Andreas Jansson");
    mono_descriptor->Copyright = strdup("GPL-3.0");
    mono_descriptor->PortCount = 4;

    port_descriptors
      = (LADSPA_PortDescriptor *)calloc(4, sizeof(LADSPA_PortDescriptor));
    mono_descriptor->PortDescriptors
      = (const LADSPA_PortDescriptor *)port_descriptors;
    port_descriptors[BW_CONTROL_L] = LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
    port_descriptors[FREQ_CONTROL_L] = LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
    port_descriptors[INPUT_L] = LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO;
    port_descriptors[OUTPUT_L] = LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO;

    port_names = (char **)calloc(4, sizeof(char *));
    mono_descriptor->PortNames = (const char **)port_names;
    port_names[BW_CONTROL_L] = strdup("Bandwidth");
    port_names[FREQ_CONTROL_L] = strdup("Frequency");
    port_names[INPUT_L] = strdup("Input");
    port_names[OUTPUT_L] = strdup("Output");

    port_range_hints =
      ((LADSPA_PortRangeHint *)calloc(4, sizeof(LADSPA_PortRangeHint)));
    mono_descriptor->PortRangeHints =
      (const LADSPA_PortRangeHint *)port_range_hints;
    port_range_hints[BW_CONTROL_L].HintDescriptor =
      (LADSPA_HINT_BOUNDED_BELOW
       | LADSPA_HINT_BOUNDED_ABOVE
       | LADSPA_HINT_LOGARITHMIC
       | LADSPA_HINT_INTEGER
       | LADSPA_HINT_DEFAULT_LOW);
    port_range_hints[BW_CONTROL_L].LowerBound = 1;
    port_range_hints[BW_CONTROL_L].UpperBound = 20000;
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
    stereo_descriptor->UniqueID = 0x00654326;
    stereo_descriptor->Label = strdup("reson_stereo");
    stereo_descriptor->Properties = LADSPA_PROPERTY_HARD_RT_CAPABLE;
    stereo_descriptor->Name = strdup("Two-pole reson filter (stereo)");
    stereo_descriptor->Maker = strdup("Andreas Jansson");
    stereo_descriptor->Copyright = strdup("GPL-3.0");
    stereo_descriptor->PortCount = 8;

    port_descriptors
      = (LADSPA_PortDescriptor *)calloc(8, sizeof(LADSPA_PortDescriptor));
    stereo_descriptor->PortDescriptors
      = (const LADSPA_PortDescriptor *)port_descriptors;
    port_descriptors[BW_CONTROL_L] = LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
    port_descriptors[FREQ_CONTROL_L] = LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
    port_descriptors[BW_CONTROL_R] = LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
    port_descriptors[FREQ_CONTROL_R] = LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
    port_descriptors[INPUT_L] = LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO;
    port_descriptors[OUTPUT_L] = LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO;
    port_descriptors[INPUT_R] = LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO;
    port_descriptors[OUTPUT_R] = LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO;

    port_names = (char **)calloc(8, sizeof(char *));
    stereo_descriptor->PortNames = (const char **)port_names;
    port_names[BW_CONTROL_L] = strdup("Bandwidth Left");
    port_names[FREQ_CONTROL_L] = strdup("Frequency Left");
    port_names[BW_CONTROL_R] = strdup("Bandwidth Right");
    port_names[FREQ_CONTROL_R] = strdup("Frequency Right");
    port_names[INPUT_L] = strdup("Input Left");
    port_names[OUTPUT_L] = strdup("Output Left");
    port_names[INPUT_R] = strdup("Input Right");
    port_names[OUTPUT_R] = strdup("Output Right");

    port_range_hints =
      ((LADSPA_PortRangeHint *)calloc(8, sizeof(LADSPA_PortRangeHint)));
    stereo_descriptor->PortRangeHints =
      (const LADSPA_PortRangeHint *)port_range_hints;
    port_range_hints[BW_CONTROL_L].HintDescriptor =
      (LADSPA_HINT_BOUNDED_BELOW
       | LADSPA_HINT_BOUNDED_ABOVE
       | LADSPA_HINT_LOGARITHMIC
       | LADSPA_HINT_INTEGER
       | LADSPA_HINT_DEFAULT_LOW);
    port_range_hints[BW_CONTROL_L].LowerBound = 1;
    port_range_hints[BW_CONTROL_L].UpperBound = 20000;
    port_range_hints[FREQ_CONTROL_L].HintDescriptor =
      (LADSPA_HINT_BOUNDED_BELOW
       | LADSPA_HINT_BOUNDED_ABOVE
       | LADSPA_HINT_LOGARITHMIC
       | LADSPA_HINT_INTEGER
       | LADSPA_HINT_DEFAULT_LOW);
    port_range_hints[FREQ_CONTROL_L].LowerBound = 20;
    port_range_hints[FREQ_CONTROL_L].UpperBound = 20000;
    port_range_hints[BW_CONTROL_R].HintDescriptor =
      (LADSPA_HINT_BOUNDED_BELOW
       | LADSPA_HINT_BOUNDED_ABOVE
       | LADSPA_HINT_LOGARITHMIC
       | LADSPA_HINT_INTEGER
       | LADSPA_HINT_DEFAULT_LOW);
    port_range_hints[BW_CONTROL_R].LowerBound = 1;
    port_range_hints[BW_CONTROL_R].UpperBound = 20000;
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

