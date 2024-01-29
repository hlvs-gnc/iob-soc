#include "bsp.h"
#include "iob-timer.h"
#include "iob-uart.h"

#include "iob_soc_conf.h"
#include "iob_soc_periphs.h"
#include "iob_soc_system.h"

#include "printf.h"
#include <stdbool.h>
#include <string.h>

#ifdef USE_TESTER
#include "iob-axistream-in.h"
#include "iob-axistream-out.h"
#endif

#include "mad.h"

#define INPUTBUFFERSIZE 4096
#define NBR_BYTES 1024
#define DEBUG 0
#define AUDIO_SPECS 1
#define SIM
#define CHECK_INPUT_BUFFER 0
#define SAMPLE_RESOLUTION 16

#define MAX_WORDS_AXIS 16
#define WORD_SIZE 4

char audio_file_in[] = "testcase-22050.mp2";

unsigned char input_buffer[INPUTBUFFERSIZE];

struct mad_decoder decoder;

int input_offset = 0;
int output_offset = 0;

bool newFrame = true;

int frames_decoded = 0;
int realtime_frames = 0;

int frame_size = 0;
int nbr_frames = 0;

unsigned int frame_time_us = 0, elapsed_sum = 0, halting_delta = 0;

unsigned int mpeg_file_size = 0, pcm_file_size = 0, audio_decoded_size = 0,
             bytes_sent = 0, stream_load_size = 0;

uint8_t last_percentage = 0, percentage, bytes_to_transfer;

// test file receive
unsigned char *audio_addr_in = NULL;
unsigned char *audio_addr_out = NULL;
unsigned char *audio_decoded = NULL;

uint32_t iob_read_mem(unsigned char *mem_addr, unsigned char *mem_buffer,
                      uint32_t offsetUnprocessed, uint32_t offsetConsumed,
                      uint32_t nbr_bytes);

/*
 * This is a private message structure. A generic pointer to this structure
 * is passed to each of the callback functions. Put here any data you need
 * to access from within the callbacks.
 */
struct buffer {
  unsigned char const *start;
  unsigned long length;
};

/*
 * This is the input callback. The purpose of this callback is to (re)fill
 * the stream buffer which is to be decoded.
 * It reads data from the AXI4 Stream In peripheral.
 */
static enum mad_flow input(void *data, struct mad_stream *stream) {

  uint16_t nwords = 0, i, to_be_read;
  uint32_t unprocessedData, consumedData;
  int32_t previous_sync;

  if (newFrame == true) {
#if (DEBUG == 1)
    printf("Reset timer\n");
#endif
    timer_reset();
  }

  // Calculate how much of the buffer has unprocessed data
  unprocessedData = stream->bufend - stream->next_frame;
  consumedData = stream->next_frame - stream->buffer;

#if (DEBUG == 1)
  printf("Decoder input callback function\n");

  printf("unprocessedData: %d\nconsumedData: %d\n", unprocessedData,
         consumedData);
#endif

  if ((consumedData == 0) && (unprocessedData == INPUTBUFFERSIZE)) {
#if (DEBUG == 1)
    printf("Input buffer is full and data is not being used\n");
#endif
    unprocessedData = INPUTBUFFERSIZE - 1;
    consumedData = 1;
  }

  // Shift unprocessed data to beginning of buffer
  for (i = 0; i < unprocessedData; i++)
    input_buffer[i] = input_buffer[consumedData + i];

  if ((consumedData + unprocessedData) != INPUTBUFFERSIZE)
    consumedData = INPUTBUFFERSIZE - unprocessedData;

#ifdef USE_TESTER

  uint32_t byte_stream_in[MAX_WORDS_AXIS];
  uint8_t j, k, rstrb, r_words;

  // Wait for more data to decode
  while (axistream_in_empty())
    ;

  // Fill buffer with new data
  while (!axistream_in_empty() &&
         (unprocessedData + MAX_AXIS_BYTES - 1 < INPUTBUFFERSIZE)) {

#if (DEBUG == 1)
    uart_puts("[MPEGD-SUT]: Loading compressed data to stream buffer\n");
#endif

    // Receive bytes while stream does not end (by TLAST signal), or up to 16
    // 32-bit words

    for (r_words = 0, j = 0; j < 1 && r_words < MAX_WORDS_AXIS; r_words++) {
      byte_stream_in[r_words] = axistream_in_pop(&rstrb, &j);
      // Extracting each byte and converting it to a an unsigned integer
      for (k = 0; k < WORD_SIZE; k++)
        input_buffer[r_words * WORD_SIZE + k + unprocessedData] =
            (byte_stream_in[r_words] >> (k * 8)) & 0xFF;
    }
    unprocessedData += r_words * k;
  }
#endif

#ifdef SIM

  if (stream_load_size + NBR_BYTES < mpeg_file_size)
    to_be_read = (NBR_BYTES < consumedData) ? NBR_BYTES : consumedData;
  else if (mpeg_file_size - stream_load_size < NBR_BYTES)
    to_be_read = mpeg_file_size - stream_load_size;

  if ((nwords = iob_read_mem(audio_addr_in, input_buffer, unprocessedData,
                             stream_load_size, to_be_read)) <= 0) {
    if (unprocessedData <= MAD_BUFFER_GUARD || nwords == 0) {
      return MAD_FLOW_STOP;
    }
  }

#endif

#if (CHECK_INPUT_BUFFER == 1)
  printf("nwords: %d\n\n", nwords);
  for (i = 0; i < nwords; i++) {
    printf("input_buffer[%d]: 0x%02x\n", i, input_buffer[i + unprocessedData]);
  }
#endif

  newFrame = false;

  previous_sync = stream->sync;

  // Set new buffer pointers
  mad_stream_buffer(stream, input_buffer, nwords + unprocessedData);

  if (nwords < NBR_BYTES)
    mad_stream_buffer(stream, input_buffer,
                      nwords + unprocessedData + MAD_BUFFER_GUARD);

  if (decoder.sync->stream.error != MAD_ERROR_BUFLEN) {
    stream->sync = previous_sync;
  }

  decoder.sync->stream.error = MAD_ERROR_NONE;

  stream_load_size += nwords;

  return MAD_FLOW_CONTINUE;
}

/*
 * The following utility routine performs simple rounding, clipping, and
 * scaling of MAD's high-resolution samples down to 16 bits. It does not
 * perform any dithering or noise shaping, which would be recommended to
 * obtain any exceptional audio quality. It is therefore not recommended to
 * use this routine if high-quality output is desired.
 */
static inline signed int scale(mad_fixed_t sample) {
  /* round */
  sample += (1L << (MAD_F_FRACBITS - 16));

  /* clip */
  if (sample >= MAD_F_ONE)
    sample = MAD_F_ONE - 1;
  else if (sample < -MAD_F_ONE)
    sample = -MAD_F_ONE;

  /* quantize */
  return sample >> (MAD_F_FRACBITS + 1 - 16);
}

/*
 * This is the output callback function. It is called after each frame of
 * MPEG audio data has been completely decoded. The purpose of this callback
 * is to output (or play) the decoded PCM audio.
 * It outputs frame data to the AXI4 Stream Out peripheral.
 */
static enum mad_flow output(void *data, struct mad_header const *header,
                            struct mad_pcm *pcm) {
  unsigned int nchannels, nsamples;
  mad_fixed_t const *left_ch, *right_ch;
  int i = output_offset, k, nbr_samples = 0;

  unsigned long long elapsed_clk_cycles;
  unsigned int elapsed_us;

#if (DEBUG == 1)
  printf("\nDecoder output callback function\n");
#endif

  /* pcm->samplerate contains the sampling frequency */
  nchannels = pcm->channels;
  nsamples = pcm->length;
  left_ch = pcm->samples[0];
  right_ch = pcm->samples[1];

  nchannels == 2 ? (k = 4) : (k = 2);

  newFrame = true;

  if (frames_decoded == 0) {

    frame_size = ((nsamples * header->bitrate) / 8) / header->samplerate;
    nbr_frames = (mpeg_file_size / frame_size);
    frame_time_us = ((1000.0 / pcm->samplerate) * (nsamples)) * 1000.0;

#if (AUDIO_SPECS == 1)
    printf("Audio file properties: \n");

    printf("nsamples: %d\nframe_size: %d\n"
           "nbr_frames: %d\nframe_time_us: %d\n"
           "nchannels: %d\n",
           nsamples, frame_size, nbr_frames, frame_time_us, nchannels);

    printf("bitrate: %ld\nsamplerate: %d\n", header->bitrate,
           header->samplerate);

    printf("pcm_file_size: %d\n\n", nsamples * k * nbr_frames);
#endif

    audio_addr_out = (unsigned char *)malloc(nsamples * k * nbr_frames);
  }

  while (nsamples--) {
    signed int sample;
    uint8_t sample_bytes[2];

    /* output sample(s) in 16-bit signed little-endian PCM */

    sample = scale(*left_ch++);
    sample_bytes[0] = sample & 0xff;
    sample_bytes[1] = sample >> 8 & 0xff;

#ifdef SIM
    audio_addr_out[i] = sample & 0xff;
    audio_addr_out[i + 1] = (sample >> 8) & 0xff;
#endif

    nbr_samples += 2;

#ifdef USE_TESTER
    while (axistream_out_full())
      ;
    axistream_out_push(sample_bytes, 2, nsamples < 1 && nchannels < 2);
#endif

    if (nchannels == 2) {
      sample = scale(*right_ch++);
      sample_bytes[0] = sample & 0xff;
      sample_bytes[1] = sample >> 8 & 0xff;

#ifdef SIM
      audio_addr_out[i + 2] = sample & 0xff;
      audio_addr_out[i + 3] = (sample >> 8) & 0xff;
#endif

      nbr_samples += 2;

#ifdef USE_TESTER
      while (axistream_out_full())
        ;
      axistream_out_push(sample_bytes, 2, nsamples < 1);
#endif
    }
    i += k;
  }

  elapsed_clk_cycles = timer_get_count();
  elapsed_us = (unsigned int)elapsed_clk_cycles / (FREQ / 1000000);

  printf("\n#Clock cycles: %llu\n", elapsed_clk_cycles);
  printf("Decoding time: %d us @%dMHz\n\n", elapsed_us, FREQ / 1000000);
  timer_reset();

  elapsed_sum += elapsed_us;

  if (elapsed_us <= frame_time_us)
    realtime_frames++;
  else
    halting_delta += (elapsed_us - frame_time_us);

  output_offset += nbr_samples;
  frames_decoded += 1;

  return MAD_FLOW_CONTINUE;
}

/*
 * This is the error callback function. It is called whenever a decoding
 * error occurs. The error is indicated by stream->error; the list of
 * possible MAD_ERROR_* errors can be found in the mad.h (or stream.h)
 * header file.
 */
static enum mad_flow error(void *data, struct mad_stream *stream,
                           struct mad_frame *frame) {
  struct buffer *buffer = data;

#if (ERROR_INFO == 1)
  printf("Decoding error 0x%04x (%s) at byte offset %u\n", stream->error,
         mad_stream_errorstr(stream), stream->this_frame - buffer->start);
#endif

  /* return MAD_FLOW_BREAK here to stop decoding (and propagate an error) */

  return MAD_FLOW_CONTINUE;
}

/*
 * Main function. The system waits for the start signal to be reset, then
 * when start signal is active, get the input buffer size and address with
 * encoded audio data to decode. The output buffer address defines where to
 * place the decoded data. The output buffer size and the finished signal are
 * updated at the end of the decoding process.
 */
int main() {
  int result;

  char pass_string[] = "Test passed!";
  char fail_string[] = "Test failed!";

  // Init timer
  timer_init(TIMER0_BASE);

  // Init uart0
  uart_init(UART0_BASE, FREQ / BAUD);
  printf_init(&uart_putc);

  // Console presentation
  uart_puts("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
  uart_puts("~~~      SoC-MPEG-D       ~~~");
  uart_puts("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

  /* configure input, output, and error functions */
  mad_decoder_init(&decoder, 0 /* private message struct */, input,
                   0 /* header */, 0 /* filter */, output, error,
                   0 /* message */);

  uart_puts("Prepare to load compressed audio sample\n");

  uint8_t last_percentage = 0, percentage, bytes_to_transfer;

  mpeg_file_size = uart_recvfile(audio_file_in, audio_addr_in);

  audio_addr_out = audio_addr_in + mpeg_file_size;

  // Start decoding data (This function will never return)
  result = mad_decoder_run(&decoder, MAD_DECODER_MODE_SYNC);

#ifdef USE_TESTER

  printf("Prepare to load decoded audio sample\n\n");

  pcm_file_size = uart_recvfile("testcase.pcm", pcm_audio);

  axistream_in_init(AXISTREAMIN0_BASE);
  axistream_out_init(AXISTREAMOUT0_BASE, 4);

  uart_puts("AXISTREAMIN0  initialization\n");
  uart_puts("AXISTREAMOUT0 initialization\n");

  // Decode audio
  while (audio_decoded_size < pcm_file_size) {

    // Try to receive more pcm data if there is any
    if (!axistream_in_empty()) {
      // Receive next byte
      axistream_in_pop(audio_decoded + audio_decoded_size, &bytes_to_transfer);

      // Check if bytes decoded are as expected
      for (i = 0; i < bytes_to_transfer; i++) {
        if (audio_decoded[audio_decoded_size + i] !=
            pcm_audio[audio_decoded_size + i]) {
          printf("\nTest failed: Decoded byte at 0x%x with value 0x%x is "
                 "different from PCM file value 0x%x!\n\n",
                 audio_decoded_size + i, audio_decoded[audio_decoded_size + i],
                 pcm_audio[audio_decoded_size + i]);
          uart_finish();
          return -1;
        }
        printf("R[%d]:%x\n", audio_decoded_size + i,
               audio_decoded[audio_decoded_size + i]); // DEBUG
      }
      audio_decoded_size += bytes_to_transfer;

      // Print progress
      if ((percentage = audio_decoded_size * 10 / pcm_file_size) >
          last_percentage) {
        printf("%3d %%\n", percentage * 10);
        last_p free(audio_decoded);
        ercentage = percentage;
      }
    }
  }
  free(audio_decoded);

#endif

  uart_puts("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
  printf("Decoded audio frames: %.1f%%\nReal-time decoded frames: %.1f%%\n",
         ((float)frames_decoded / nbr_frames) * 100,
         ((float)realtime_frames / nbr_frames) * 100);
  printf("Average decoding time per frame: %d us\n",
         elapsed_sum / frames_decoded);
  printf("Halting difference per frame: %d us", halting_delta / frames_decoded);
  uart_puts("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

  uart_sendfile("out-testcase.pcm", output_offset, audio_addr_out);

  free(audio_addr_in);
  free(audio_addr_out);

  uart_puts("\nTest complete.\n\n");

  uart_sendfile("test.log", strlen(pass_string), pass_string);

  // End UART0 connection
  uart_finish();
}

uint32_t iob_read_mem(unsigned char *mem_addr, unsigned char *mem_buffer,
                      uint32_t offsetUnprocessed, uint32_t offsetConsumed,
                      uint32_t nbr_bytes) {

  int i = 0;

  while (i < nbr_bytes) {
    mem_buffer[i + offsetUnprocessed] = mem_addr[i + offsetConsumed];
    i++;
  }

  return i;
}
