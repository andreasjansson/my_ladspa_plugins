/* 
 * iir.c - A simple one-term finite impulse response filter
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
 * A simple 1-pole IIR filter that is both a low pass and a high pass
 * filter. If the coefficient is positive, it is low pass, if it is
 * negative it is high pass. At coef = 0, the signal is unaffected.
 */

#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "ladspa.h"

#define MIN_FREQ 1

// The port numbers for the plugin
#define COEF_CONTROL_L 0
#define INPUT_L        1
#define OUTPUT_L       2

#define COEF_CONTROL_R 3
#define INPUT_R        4
#define OUTPUT_R       5

LADSPA_Descriptor *mono_descriptor = NULL;
LADSPA_Descriptor *stereo_descriptor = NULL;

/**
 * Structure to hold connections and state.
 */
typedef struct {
  LADSPA_Data *coef_control_value_l;
  LADSPA_Data *input_buffer_l;
  LADSPA_Data *output_buffer_l;

  LADSPA_Data *coef_control_value_r;
  LADSPA_Data *input_buffer_r;
  LADSPA_Data *output_buffer_r;

  // a one-sample buffer that holds the value of
  // the previously output sample
  LADSPA_Data previous_sample_l;
  LADSPA_Data previous_sample_r;
} filter_type;

/**
 * Construct a new plugin instance.
 */
LADSPA_Handle instantiate_filter(const LADSPA_Descriptor *descriptor,
                                 unsigned long sample_rate)
{
  return malloc(sizeof(filter_type));
}

void activate_filter(LADSPA_Handle instance)
{
  filter_type *filter = (filter_type *)instance;
  filter->previous_sample_l = 0;
  filter->previous_sample_r = 0;
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
  case COEF_CONTROL_L:
    filter->coef_control_value_l = data_location;
    break;
  case COEF_CONTROL_R:
    filter->coef_control_value_r = data_location;
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
inline void run_filter(LADSPA_Handle instance, unsigned long sample_count,
                       int stereo)
{
  filter_type *filter = (filter_type *)instance;
  LADSPA_Data *input_l = filter->input_buffer_l;
  LADSPA_Data *input_r = stereo ? filter->input_buffer_r : NULL;
  LADSPA_Data *output_l = filter->output_buffer_l;
  LADSPA_Data *output_r = stereo ? filter->output_buffer_r : NULL;
  LADSPA_Data coef_l = *filter->coef_control_value_l;
  LADSPA_Data coef_r = stereo ? *filter->coef_control_value_r : 0;

  while(sample_count -- > 0) {

    // add the current input sample to the previous output sample,
    // times a coefficient. normalise so that peak amplitude is always 1.
    // the multiplication by powf(fabs(1 - coef_l), 2) is probaby wrong -
    // i'll come back to that later.
    *output_l = *input_l * (1 - fabs(coef_l)) +
      filter->previous_sample_l * coef_l;

    filter->previous_sample_l = *output_l;

    if(stereo) {
      *output_r = *input_r * (1 - fabs(coef_r)) +
        filter->previous_sample_r * coef_r;

      filter->previous_sample_r = *output_r;
    }      

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
    mono_descriptor->UniqueID = 0x00654323;
    mono_descriptor->Label = strdup("iir_mono");
    mono_descriptor->Properties = LADSPA_PROPERTY_HARD_RT_CAPABLE;
    mono_descriptor->Name = strdup("One-pole IIR filter (mono)");
    mono_descriptor->Maker = strdup("Andreas Jansson");
    mono_descriptor->Copyright = strdup("GPL-3.0");
    mono_descriptor->PortCount = 3;

    port_descriptors
      = (LADSPA_PortDescriptor *)calloc(3, sizeof(LADSPA_PortDescriptor));
    mono_descriptor->PortDescriptors
      = (const LADSPA_PortDescriptor *)port_descriptors;
    port_descriptors[COEF_CONTROL_L] = LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
    port_descriptors[INPUT_L] = LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO;
    port_descriptors[OUTPUT_L] = LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO;

    port_names = (char **)calloc(3, sizeof(char *));
    mono_descriptor->PortNames = (const char **)port_names;
    port_names[COEF_CONTROL_L] = strdup("Coefficient");
    port_names[INPUT_L] = strdup("Input");
    port_names[OUTPUT_L] = strdup("Output");

    port_range_hints =
      ((LADSPA_PortRangeHint *)calloc(3, sizeof(LADSPA_PortRangeHint)));
    mono_descriptor->PortRangeHints =
      (const LADSPA_PortRangeHint *)port_range_hints;
    port_range_hints[COEF_CONTROL_L].HintDescriptor =
      (LADSPA_HINT_BOUNDED_BELOW
       | LADSPA_HINT_BOUNDED_ABOVE
       | LADSPA_HINT_DEFAULT_0);
    port_range_hints[COEF_CONTROL_L].LowerBound = -.99999;
    port_range_hints[COEF_CONTROL_L].UpperBound = .99999;
    port_range_hints[INPUT_L].HintDescriptor = 0;
    port_range_hints[OUTPUT_L].HintDescriptor = 0;

    mono_descriptor->instantiate = instantiate_filter;
    mono_descriptor->connect_port = connect_port_to_filter;
    mono_descriptor->activate = activate_filter;
    mono_descriptor->run = run_mono_filter;
    mono_descriptor->run_adding = NULL;
    mono_descriptor->set_run_adding_gain = NULL;
    mono_descriptor->deactivate = NULL;
    mono_descriptor->cleanup = cleanup_filter;
  }

  if(stereo_descriptor) {
    stereo_descriptor->UniqueID = 0x00654324;
    stereo_descriptor->Label = strdup("iir_stereo");
    stereo_descriptor->Properties = LADSPA_PROPERTY_HARD_RT_CAPABLE;
    stereo_descriptor->Name = strdup("One-pole IIR filter (stereo)");
    stereo_descriptor->Maker = strdup("Andreas Jansson");
    stereo_descriptor->Copyright = strdup("GPL-3.0");
    stereo_descriptor->PortCount = 6;

    port_descriptors
      = (LADSPA_PortDescriptor *)calloc(6, sizeof(LADSPA_PortDescriptor));
    stereo_descriptor->PortDescriptors
      = (const LADSPA_PortDescriptor *)port_descriptors;
    port_descriptors[COEF_CONTROL_L] = LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
    port_descriptors[INPUT_L] = LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO;
    port_descriptors[OUTPUT_L] = LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO;
    port_descriptors[COEF_CONTROL_R] = LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
    port_descriptors[INPUT_R] = LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO;
    port_descriptors[OUTPUT_R] = LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO;

    port_names = (char **)calloc(6, sizeof(char *));
    stereo_descriptor->PortNames = (const char **)port_names;
    port_names[COEF_CONTROL_L] = strdup("Coefficient Left");
    port_names[INPUT_L] = strdup("Input Left");
    port_names[OUTPUT_L] = strdup("Output Left");
    port_names[COEF_CONTROL_R] = strdup("Coefficient Right");
    port_names[INPUT_R] = strdup("Input Right");
    port_names[OUTPUT_R] = strdup("Output Right");

    port_range_hints =
      ((LADSPA_PortRangeHint *)calloc(6, sizeof(LADSPA_PortRangeHint)));
    stereo_descriptor->PortRangeHints =
      (const LADSPA_PortRangeHint *)port_range_hints;
    port_range_hints[COEF_CONTROL_L].HintDescriptor =
      (LADSPA_HINT_BOUNDED_BELOW
       | LADSPA_HINT_BOUNDED_ABOVE
       | LADSPA_HINT_DEFAULT_0);
    port_range_hints[COEF_CONTROL_L].LowerBound = -.99999;
    port_range_hints[COEF_CONTROL_L].UpperBound = .99999;
    port_range_hints[INPUT_L].HintDescriptor = 0;
    port_range_hints[OUTPUT_L].HintDescriptor = 0;
    port_range_hints[COEF_CONTROL_R].HintDescriptor =
      (LADSPA_HINT_BOUNDED_BELOW
       | LADSPA_HINT_BOUNDED_ABOVE
       | LADSPA_HINT_DEFAULT_0);
    port_range_hints[COEF_CONTROL_R].LowerBound = -.99999;
    port_range_hints[COEF_CONTROL_R].UpperBound = .99999;
    port_range_hints[INPUT_R].HintDescriptor = 0;
    port_range_hints[OUTPUT_R].HintDescriptor = 0;

    stereo_descriptor->instantiate = instantiate_filter;
    stereo_descriptor->connect_port = connect_port_to_filter;
    stereo_descriptor->activate = activate_filter;
    stereo_descriptor->run = run_stereo_filter;
    stereo_descriptor->run_adding = NULL;
    stereo_descriptor->set_run_adding_gain = NULL;
    stereo_descriptor->deactivate = NULL;
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

