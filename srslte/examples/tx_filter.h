#ifndef _TX_FILTER_H_
#define _TX_FILTER_H_

#define NOF_MULS_MM128 4

typedef struct {
  __attribute__ ((aligned (16))) __m128 *filter_mm128_kernel;
  uint32_t filter_length;
} filter_mm128_context_t;

void initialize_filter_mm128(float *coeffs, uint32_t filter_length);

void unitialize_filter_mm128();

void load_simd_kernel_mm128(float *coeffs);

//-------------------------------------------
#define NOF_MULS_MM256 8

#define NOF_SAMPLES_TO_READ 2*NOF_MULS_MM256

typedef struct {
  __attribute__ ((aligned (32))) __m256 *filter_mm256_kernel;
  uint32_t filter_length;
} filter_mm256_context_t;

void initialize_filter_mm256(float *coeffs, uint32_t filter_length);

void unitialize_filter_mm256();

void load_simd_kernel_mm256(float *coeffs);

void print_mm256_vector(__m256 *prod);

void print_mm256_filter_kernel(__m256 *prod, uint32_t filter_length);

// in_len: Number of complex samples and not number of pairs of I and Q parts.
inline void avxFirFilter(cf_t *cplx_in, int in_len, __m256 *filter_kernel, int filter_len, cf_t *cplx_out) {

  int M = 8;
  __attribute__(( aligned(32))) __m256 block_0, block_1, block_I, block_Q;
  __attribute__(( aligned(32))) __m256 dp_I, res_I, dp_Q, res_Q;

  // Static initializations.
  __m256i mask = _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7);
  const int imm8_I = _MM_SHUFFLE(2, 0, 2, 0);
  const int imm8_Q = _MM_SHUFFLE(3, 1, 3, 1);
  // Cast complex samples type into single precision floating-point type.
  // Note that one complex sample becomes 2 floating-point values.
  __attribute__ ((aligned (32))) float *in = (float*)cplx_in;
  __attribute__ ((aligned (32))) float *out = (float*)cplx_out;

  int cnt = 0, nof_outer_loop_iter = in_len/M;

  // Outer loop.
  for(int k = 0; k < nof_outer_loop_iter; k++) {

    // Load 8 values from the input vector.
    block_0 = _mm256_load_ps(in+(k*2*M));
    // Load the next 8 values from the input vector.
    block_1 = _mm256_load_ps(in+(k*2*M)+M);
    // Shuffle the samples so that we have 8 real elements only in this vector, however, they are not in time sequence.
    block_I = _mm256_shuffle_ps(block_0, block_1, imm8_I);
    // Organize the real samples so that they are in time sequence.
    block_I = _mm256_permutevar8x32_ps(block_I, mask);
    // Shuffle the samples so that we have 8 imaginary elements only in this vector, however, they are not in sequence.
    block_Q = _mm256_shuffle_ps(block_0, block_1, imm8_Q);
    // Organize the imaginary samples so that they are in time sequence.
    block_Q = _mm256_permutevar8x32_ps(block_Q, mask);

    cnt = 0;
    // Inner loop.
    for(int i = 0; i < (filter_len + M - 1); i++) {

      // Multiply 4 real elements in the lower part of the vector and sum the 4 products.
      // Simulaneoulsy does the same for the higher 4 elements.
      dp_I = _mm256_dp_ps(filter_kernel[i], block_I, 0xff);
      // Swap lower 4 elements with higher 4 elements.
      res_I = _mm256_permute2f128_ps(dp_I, dp_I, 1);
      // Sum the elements so that we have dot product of eight real elements in the index 0 of res_I.
      res_I = _mm256_add_ps(res_I, dp_I);

      // Multiply 4 imaginary elements in the lower part of the vector and sum the 4 products.
      // Simulaneoulsy does the same for the higher 4 elements.
      dp_Q = _mm256_dp_ps(filter_kernel[i], block_Q, 0xff);
      // Swap lower 4 elements with higher 4 elements.
      res_Q = _mm256_permute2f128_ps(dp_Q, dp_Q, 1);
      // Sum the elements so that we have dot product of eight imaginary elements in the index 0 of res_Q.
      res_Q = _mm256_add_ps(res_Q, dp_Q);

      // Accumulates the results in the output vector. Note that 2 values (real and imaginary) are produced at each iteration.
      out[k*16 + cnt] += res_I[0];
      out[k*16 + cnt + 1] += res_Q[0];
      cnt += 2;
    }
  }
}

#endif // _TX_FILTER_H_
