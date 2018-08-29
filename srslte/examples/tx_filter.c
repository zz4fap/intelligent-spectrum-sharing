#include <immintrin.h>
#include <stdlib.h>

#include "srslte/srslte.h"

#include "tx_filter.h"

static float filter_coeffs_16[17] __attribute__ ((aligned (32)))  = {
  0.00000000000000000000e+00,	-6.40574149210236531982e-03,	1.33542201119032678813e-02,	-7.59039005902627759992e-03,	-2.20932420507254541342e-02,
  7.62999348843487190663e-02,	-1.41565981761304526820e-01,	1.95126718051850223112e-01,	7.85748964630113122531e-01,	1.95126718051850223112e-01,
  -1.41565981761304526820e-01,	7.62999348843487190663e-02,	-2.20932420507254541342e-02,	-7.59039005902627759992e-03,	1.33542201119032678813e-02,
  -6.40574149210236531982e-03,	0.00000000000000000000e+00
};

static float filter_coeffs_8[9] __attribute__ ((aligned (32)))  = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};

static float filter_coeffs_4[5] __attribute__ ((aligned (32)))  = {1.0, 2.0, 3.0, 4.0, 5.0};

void naive_filter(cf_t* input, uint32_t input_length, float* coeffs, uint32_t coeff_len, cf_t* output) {
  int pos = 0;
  for(int n = 0; n < (input_length+coeff_len-1); n++) {
    for(int i = 0; i < coeff_len; i++) {
      pos = (n-i);
      if(pos >= 0 && pos < input_length) {
        output[n] = output[n] + (input[pos]*coeffs[i]);
      }
    }
  }
}

void sse256_filter() {

  float f1[8] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};

  float f2[8] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};

  float f3[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  float output[8];

  __m256 s1 = _mm256_load_ps(f1);

  __m256 s2 = _mm256_load_ps(f2);

  __m256 s3 = _mm256_load_ps(f3);

  __m256 prod __attribute__ ((aligned (32)));

  prod = _mm256_fmadd_ps(s1, s2, s3);

  _mm256_storeu_ps(output, prod);

  for(int i = 0; i < 8; i++) {
    printf("%f\n",output[i]);
  }
}

void print_result(cf_t* output, uint32_t output_length, float* filter_coeffs) {
  for(uint32_t i = 0; i < output_length; i++) {
    printf("output[%d]: (%e,%e) - filter_coeffs[%d]: %e\n",i, __real__ output[i], __imag__ output[i], i, filter_coeffs[i]);
  }
}

void test_naive() {
  uint32_t filter_length = 17;

  uint32_t input_length = 20;
  __attribute__ ((aligned (32))) cf_t input[input_length];
  bzero(input, sizeof(cf_t)*input_length);
  input[0] = 1.0+0.0i;

  uint32_t output_length = input_length+filter_length-1;
  __attribute__ ((aligned (32))) cf_t output[output_length];
  bzero(output, sizeof(cf_t)*output_length);

  naive_filter(input, input_length, filter_coeffs_16, filter_length, output);

  print_result(output, output_length, filter_coeffs_16);
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

__attribute__ ((aligned (16))) filter_mm128_context_t *filter_mm128_context = NULL;

void initialize_filter_mm128(float *coeffs, uint32_t filter_length) {
  // Allocate memory for filter context.
  filter_mm128_context = (filter_mm128_context_t*)srslte_vec_malloc(sizeof(filter_mm128_context_t));
  // Check if memory allocation was correctly done.
  if(filter_mm128_context == NULL) {
    printf("Error allocating memory for filter context.\n");
    exit(-1);
  }
  // Allocate memory for filter kernel.
  filter_mm128_context->filter_mm128_kernel = (__m128 *)srslte_vec_malloc(filter_length*sizeof(__m128));
  // Check if memory allocation was correctly done.
  if(filter_mm128_context->filter_mm128_kernel == NULL) {
    printf("Error allocating memory for filter kernel.\n");
    exit(-1);
  }
  // Set filter length.
  filter_mm128_context->filter_length = filter_length;
  // Create filter kernel.
  load_simd_kernel_mm128(coeffs);
}

void unitialize_filter_mm128() {
  if(filter_mm128_context->filter_mm128_kernel != NULL) {
    free(filter_mm128_context->filter_mm128_kernel);
  }
  if(filter_mm128_context != NULL) {
    free(filter_mm128_context);
  }
}

void load_simd_kernel_mm128(float *coeffs) {
  float kernel_row[NOF_MULS_MM128] __attribute__ ((aligned (16)));
  int coeff_idx = 0;
  for(int i = 0; i < (NOF_MULS_MM128+filter_mm128_context->filter_length-1); i++) {
    coeff_idx = i;
    for(int k = 0; k < NOF_MULS_MM128; k++) {
      if(coeff_idx >= 0 && coeff_idx < filter_mm128_context->filter_length) {
        kernel_row[k] = coeffs[coeff_idx];
      } else {
        kernel_row[k] = 0.0;
      }
      coeff_idx--;
    }
    // Load four single-precision, floating-point values.
    filter_mm128_context->filter_mm128_kernel[i] = _mm_load_ps(kernel_row);
  }
}

void execute_filter_mm128(cf_t *input_complex, uint32_t input_length, cf_t *output_complex) {

  __m128 signal_block __attribute__ ((aligned (16)));
  __m128 prod __attribute__ ((aligned (16)));
  __m128 acc __attribute__ ((aligned (16)));

  __attribute__ ((aligned (16))) float *input = (float*)input_complex;
  __attribute__ ((aligned (16))) float *output = (float*)output_complex;

  int nof_outer_loop_iter = input_length/NOF_MULS_MM128;

  for(int k = 0; k < nof_outer_loop_iter; k++) {

    signal_block = _mm_loadu_ps(input+(k*NOF_MULS_MM128));

    // Clears the four single-precision, floating-point values.
    acc = _mm_setzero_ps();

    for(int i = 0; i < 2; i++) {

      prod = _mm_mul_ps(filter_mm128_context->filter_mm128_kernel[i], signal_block);

      // Accumulate the 4 parallel values.
      acc = _mm_add_ps(acc, prod);

    }
  }


  // Stores four single-precision, floating-point values.
  _mm_storeu_ps(output, acc);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

__attribute__ ((aligned (32))) filter_mm256_context_t *filter_mm256_context = NULL;

void initialize_filter_mm256(float *coeffs, uint32_t filter_length) {
  // Allocate memory for filter context.
  filter_mm256_context = (filter_mm256_context_t*)srslte_vec_malloc(sizeof(filter_mm256_context_t));
  // Check if memory allocation was correctly done.
  if(filter_mm256_context == NULL) {
    printf("Error allocating memory for filter context.\n");
    exit(-1);
  }
  // Allocate memory for filter kernel.
  filter_mm256_context->filter_mm256_kernel = (__m256 *)srslte_vec_malloc((filter_length+NOF_MULS_MM256-1)*sizeof(__m256));
  // Check if memory allocation was correctly done.
  if(filter_mm256_context->filter_mm256_kernel == NULL) {
    printf("Error allocating memory for filter kernel.\n");
    exit(-1);
  }
  // Set filter length.
  filter_mm256_context->filter_length = filter_length;
  // Create filter kernel.
  load_simd_kernel_mm256(coeffs);
}

void unitialize_filter_mm256() {
  if(filter_mm256_context->filter_mm256_kernel != NULL) {
    free(filter_mm256_context->filter_mm256_kernel);
  }
  if(filter_mm256_context != NULL) {
    free(filter_mm256_context);
  }
}

void load_simd_kernel_mm256(float *coeffs) {
  float kernel_row[NOF_MULS_MM256] __attribute__ ((aligned (32)));
  int coeff_idx = 0;
  for(int i = 0; i < (NOF_MULS_MM256+filter_mm256_context->filter_length-1); i++) {
    coeff_idx = i;
    for(int k = 0; k < NOF_MULS_MM256; k++) {
      if(coeff_idx >= 0 && coeff_idx < filter_mm256_context->filter_length) {
        kernel_row[k] = coeffs[coeff_idx];
      } else {
        kernel_row[k] = 0.0;
      }
      coeff_idx--;
    }
    // Load four single-precision, floating-point values.
    filter_mm256_context->filter_mm256_kernel[i] = _mm256_load_ps(kernel_row);
  }
}

void execute_filter_naive(cf_t* input, uint32_t input_length, float* coeffs, uint32_t coeff_len, cf_t* output) {
  int pos = 0;
  for(int n = 0; n < (input_length+coeff_len-1); n++) {
    for(int i = 0; i < coeff_len; i++) {
      pos = (n-i);
      if(pos >= 0 && pos < input_length) {
        output[n] = output[n] + (input[pos]*coeffs[i]);
      }
    }
  }
}

// input_length: Number of complex samples and not number of pairs of I and Q parts.
void execute_filter_mm256_v0(cf_t *input_complex, uint32_t input_length, cf_t *output_complex) {

  __m256 signal_block_0 __attribute__ ((aligned (32)));
  __m256 signal_block_1 __attribute__ ((aligned (32)));
  __m256 signal_block_I __attribute__ ((aligned (32)));
  __m256 signal_block_Q __attribute__ ((aligned (32)));
  __m256 dp_high_I __attribute__ ((aligned (32)));
  __m256 dp_low_I __attribute__ ((aligned (32)));
  __m256 prod_I __attribute__ ((aligned (32)));
  __m256 dp_high_Q __attribute__ ((aligned (32)));
  __m256 dp_low_Q __attribute__ ((aligned (32)));
  __m256 prod_Q __attribute__ ((aligned (32)));

  __attribute__ ((aligned (32))) float *input = (float*)input_complex;
  __attribute__ ((aligned (32))) float *output = (float*)output_complex;

  int cnt;
  int nof_outer_loop_iter = input_length/NOF_MULS_MM256;

  for(int k = 0; k < nof_outer_loop_iter; k++) {

    signal_block_0 = _mm256_loadu_ps(input+(k*NOF_MULS_MM256*2));

    signal_block_1 = _mm256_loadu_ps(input+NOF_MULS_MM256+(k*NOF_MULS_MM256*2));

    signal_block_I = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(2, 0, 2, 0));

    signal_block_I = _mm256_permutevar8x32_ps(signal_block_I, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    signal_block_Q = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(3, 1, 3, 1));

    signal_block_Q = _mm256_permutevar8x32_ps(signal_block_Q, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    cnt = 0;
    for(int i = 0; i < (filter_mm256_context->filter_length + NOF_MULS_MM256 - 1); i++) {

      // Multiply eight elements of I.
      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_I, 0xff);

      dp_low_I = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);

      prod_I = _mm256_add_ps(dp_low_I, dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_Q, 0xff);

      dp_low_Q = _mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1);

      prod_Q = _mm256_add_ps(dp_low_Q, dp_high_Q);

      // Accumulates the result in the output vector.
      output[k*2*NOF_MULS_MM256 + cnt] += prod_I[0];
      output[k*2*NOF_MULS_MM256 + (cnt+1)] += prod_Q[0];
      cnt += 2;
    }
  }
}

// input_length: Number of complex samples and not number of pairs of I and Q parts.
void execute_filter_mm256_v1(cf_t *input_complex, uint32_t input_length, cf_t *output_complex) {

  __m256 signal_block_0 __attribute__ ((aligned (32)));
  __m256 signal_block_1 __attribute__ ((aligned (32)));
  __m256 signal_block_I __attribute__ ((aligned (32)));
  __m256 signal_block_Q __attribute__ ((aligned (32)));
  __m256 dp_high_I __attribute__ ((aligned (32)));
  __m256 dp_high_Q __attribute__ ((aligned (32)));

  __attribute__ ((aligned (32))) float *input = (float*)input_complex;
  __attribute__ ((aligned (32))) float *output = (float*)output_complex;

  int cnt;
  int nof_outer_loop_iter = input_length/NOF_MULS_MM256;

  for(int k = 0; k < nof_outer_loop_iter; k++) {

    signal_block_0 = _mm256_loadu_ps(input+(k*NOF_MULS_MM256*2));

    signal_block_1 = _mm256_loadu_ps(input+NOF_MULS_MM256+(k*NOF_MULS_MM256*2));

    signal_block_I = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(2, 0, 2, 0));

    signal_block_I = _mm256_permutevar8x32_ps(signal_block_I, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    signal_block_Q = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(3, 1, 3, 1));

    signal_block_Q = _mm256_permutevar8x32_ps(signal_block_Q, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    cnt = 0;
    for(int i = 0; i < (filter_mm256_context->filter_length + NOF_MULS_MM256 - 1); i++) {

      // Multiply eight elements of I.
      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_I, 0xff);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_Q, 0xff);

      // Accumulates the result in the output vector.
      output[k*2*NOF_MULS_MM256 + cnt] += (dp_high_I[0] + dp_high_I[4]);
      output[k*2*NOF_MULS_MM256 + (cnt+1)] += (dp_high_Q[0] + dp_high_Q[4]);
      cnt += 2;
    }
  }
}

// input_length: Number of complex samples and not number of pairs of I and Q parts.
void execute_filter_mm256_v2(cf_t *input_complex, uint32_t input_length, cf_t *output_complex) {

  __m256 signal_block_0 __attribute__ ((aligned (32)));
  __m256 signal_block_1 __attribute__ ((aligned (32)));
  __m256 signal_block_I __attribute__ ((aligned (32)));
  __m256 signal_block_Q __attribute__ ((aligned (32)));
  __m256 dp_high_I __attribute__ ((aligned (32)));
  __m256 dp_low_I __attribute__ ((aligned (32)));
  __m256 prod_I __attribute__ ((aligned (32)));
  __m256 dp_high_Q __attribute__ ((aligned (32)));
  __m256 dp_low_Q __attribute__ ((aligned (32)));
  __m256 prod_Q __attribute__ ((aligned (32)));

  __attribute__ ((aligned (32))) float *input = (float*)input_complex;
  __attribute__ ((aligned (32))) float *output = (float*)output_complex;

  int cnt;
  int nof_outer_loop_iter = input_length/NOF_MULS_MM256;

  for(int k = 0; k < nof_outer_loop_iter; k++) {

    signal_block_0 = _mm256_load_ps(input+(k*NOF_MULS_MM256*2));

    signal_block_1 = _mm256_load_ps(input+NOF_MULS_MM256+(k*NOF_MULS_MM256*2));

    signal_block_I = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(2, 0, 2, 0));

    signal_block_I = _mm256_permutevar8x32_ps(signal_block_I, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    signal_block_Q = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(3, 1, 3, 1));

    signal_block_Q = _mm256_permutevar8x32_ps(signal_block_Q, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    cnt = 0;
    for(int i = 0; i < (filter_mm256_context->filter_length + NOF_MULS_MM256 - 1); i++) {

      // Multiply eight elements of I.
      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_I, 0xff);

      dp_low_I = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);

      prod_I = _mm256_add_ps(dp_low_I, dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_Q, 0xff);

      dp_low_Q = _mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1);

      prod_Q = _mm256_add_ps(dp_low_Q, dp_high_Q);

      // Accumulates the result in the output vector.
      output[k*2*NOF_MULS_MM256 + cnt] += prod_I[0];
      output[k*2*NOF_MULS_MM256 + (cnt+1)] += prod_Q[0];
      cnt += 2;
    }
  }
}

// input_length: Number of complex samples and not number of pairs of I and Q parts.
void execute_filter_mm256_v3(cf_t *input_complex, uint32_t input_length, cf_t *output_complex) {

  __m256 signal_block_0 __attribute__ ((aligned (32)));
  __m256 signal_block_1 __attribute__ ((aligned (32)));
  __m256 signal_block_I __attribute__ ((aligned (32)));
  __m256 signal_block_Q __attribute__ ((aligned (32)));
  __m256 dp_high_I __attribute__ ((aligned (32)));
  __m256 dp_low_I __attribute__ ((aligned (32)));
  __m256 prod_I __attribute__ ((aligned (32)));
  __m256 dp_high_Q __attribute__ ((aligned (32)));
  __m256 dp_low_Q __attribute__ ((aligned (32)));
  __m256 prod_Q __attribute__ ((aligned (32)));

  __attribute__ ((aligned (32))) float *input = (float*)input_complex;
  __attribute__ ((aligned (32))) float *output = (float*)output_complex;

  int cnt;
  int nof_outer_loop_iter = input_length/NOF_MULS_MM256;

  for(int k = 0; k < nof_outer_loop_iter; k++) {

    signal_block_0 = _mm256_load_ps(input+(k*NOF_MULS_MM256*2));

    signal_block_1 = _mm256_load_ps(input+NOF_MULS_MM256+(k*NOF_MULS_MM256*2));

    signal_block_I = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(2, 0, 2, 0));

    signal_block_I = _mm256_permutevar8x32_ps(signal_block_I, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    signal_block_Q = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(3, 1, 3, 1));

    signal_block_Q = _mm256_permutevar8x32_ps(signal_block_Q, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    cnt = 0;
    for(int i = 0; i < (filter_mm256_context->filter_length + NOF_MULS_MM256 - 1); i++) {

      // Multiply eight elements of I.
      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_I, 0xff);

      dp_low_I = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);

      prod_I = _mm256_add_ps(dp_low_I, dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_Q, 0xff);

      dp_low_Q = _mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1);

      prod_Q = _mm256_add_ps(dp_low_Q, dp_high_Q);

      // Accumulates the result in the output vector.
      if(k > 0) {
        output[k*2*NOF_MULS_MM256 + cnt] += prod_I[0];
        output[k*2*NOF_MULS_MM256 + (cnt+1)] += prod_Q[0];
      } else {
        output[k*2*NOF_MULS_MM256 + cnt] = prod_I[0];
        output[k*2*NOF_MULS_MM256 + (cnt+1)] = prod_Q[0];
      }
      cnt += 2;
    }
  }
}

// input_length: Number of complex samples and not number of pairs of I and Q parts.
void execute_filter_mm256_v4(cf_t *input_complex, uint32_t input_length, cf_t *output_complex) {

  __m256 signal_block_0 __attribute__ ((aligned (32)));
  __m256 signal_block_1 __attribute__ ((aligned (32)));
  __m256 signal_block_I __attribute__ ((aligned (32)));
  __m256 signal_block_Q __attribute__ ((aligned (32)));
  __m256 dp_high_I __attribute__ ((aligned (32)));
  __m256 dp_low_I __attribute__ ((aligned (32)));
  __m256 prod_I __attribute__ ((aligned (32)));
  __m256 dp_high_Q __attribute__ ((aligned (32)));
  __m256 dp_low_Q __attribute__ ((aligned (32)));
  __m256 prod_Q __attribute__ ((aligned (32)));

  __attribute__ ((aligned (32))) float *input = (float*)input_complex;
  __attribute__ ((aligned (32))) float *output = (float*)output_complex;

  int cnt;
  int nof_outer_loop_iter = input_length/NOF_MULS_MM256;
  int nof_inner_loop_iter = (filter_mm256_context->filter_length + NOF_MULS_MM256 - 1)/4;

  for(int k = 0; k < nof_outer_loop_iter; k++) {

    signal_block_0 = _mm256_load_ps(input+(k*NOF_MULS_MM256*2));

    signal_block_1 = _mm256_load_ps(input+NOF_MULS_MM256+(k*NOF_MULS_MM256*2));

    signal_block_I = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(2, 0, 2, 0));

    signal_block_I = _mm256_permutevar8x32_ps(signal_block_I, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    signal_block_Q = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(3, 1, 3, 1));

    signal_block_Q = _mm256_permutevar8x32_ps(signal_block_Q, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    cnt = 0;
    for(int i = 0; i < nof_inner_loop_iter; i++) {

      // Multiply eight elements of I.
      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_I, 0xff);
      dp_low_I = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);
      prod_I = _mm256_add_ps(dp_low_I, dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_Q, 0xff);
      dp_low_Q = _mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1);
      prod_Q = _mm256_add_ps(dp_low_Q, dp_high_Q);

      // Accumulates the result in the output vector.
      output[k*2*NOF_MULS_MM256 + cnt] += prod_I[0];
      output[k*2*NOF_MULS_MM256 + (cnt+1)] += prod_Q[0];
      cnt += 2;

      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+1], signal_block_I, 0xff);
      dp_low_I = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);
      prod_I = _mm256_add_ps(dp_low_I, dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+1], signal_block_Q, 0xff);
      dp_low_Q = _mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1);
      prod_Q = _mm256_add_ps(dp_low_Q, dp_high_Q);

      // Accumulates the result in the output vector.
      output[k*2*NOF_MULS_MM256 + cnt] += prod_I[0];
      output[k*2*NOF_MULS_MM256 + (cnt+1)] += prod_Q[0];
      cnt += 2;

      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+2], signal_block_I, 0xff);
      dp_low_I = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);
      prod_I = _mm256_add_ps(dp_low_I, dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+2], signal_block_Q, 0xff);
      dp_low_Q = _mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1);
      prod_Q = _mm256_add_ps(dp_low_Q, dp_high_Q);

      // Accumulates the result in the output vector.
      output[k*2*NOF_MULS_MM256 + cnt] += prod_I[0];
      output[k*2*NOF_MULS_MM256 + (cnt+1)] += prod_Q[0];
      cnt += 2;

      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+3], signal_block_I, 0xff);
      dp_low_I = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);
      prod_I = _mm256_add_ps(dp_low_I, dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+3], signal_block_Q, 0xff);
      dp_low_Q = _mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1);
      prod_Q = _mm256_add_ps(dp_low_Q, dp_high_Q);

      // Accumulates the result in the output vector.
      output[k*2*NOF_MULS_MM256 + cnt] += prod_I[0];
      output[k*2*NOF_MULS_MM256 + (cnt+1)] += prod_Q[0];
      cnt += 2;

    }
  }
}

// input_length: Number of complex samples and not number of pairs of I and Q parts.
void execute_filter_mm256_v5(cf_t *input_complex, uint32_t input_length, cf_t *output_complex) {

  __m256 signal_block_0 __attribute__ ((aligned (32)));
  __m256 signal_block_1 __attribute__ ((aligned (32)));
  __m256 signal_block_I __attribute__ ((aligned (32)));
  __m256 signal_block_Q __attribute__ ((aligned (32)));
  __m256 dp_high_I __attribute__ ((aligned (32)));
  __m256 dp_low_I __attribute__ ((aligned (32)));
  __m256 prod_I __attribute__ ((aligned (32)));
  __m256 dp_high_Q __attribute__ ((aligned (32)));
  __m256 dp_low_Q __attribute__ ((aligned (32)));
  __m256 prod_Q __attribute__ ((aligned (32)));

  __attribute__ ((aligned (32))) float *input = (float*)input_complex;
  __attribute__ ((aligned (32))) float *output = (float*)output_complex;

  int cnt;
  int nof_outer_loop_iter = input_length/NOF_MULS_MM256;
  int nof_inner_loop_iter = (filter_mm256_context->filter_length + NOF_MULS_MM256 - 1)/NOF_MULS_MM256;

  for(int k = 0; k < nof_outer_loop_iter; k++) {

    signal_block_0 = _mm256_load_ps(input+(k*NOF_MULS_MM256*2));

    signal_block_1 = _mm256_load_ps(input+NOF_MULS_MM256+(k*NOF_MULS_MM256*2));

    signal_block_I = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(2, 0, 2, 0));

    signal_block_I = _mm256_permutevar8x32_ps(signal_block_I, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    signal_block_Q = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(3, 1, 3, 1));

    signal_block_Q = _mm256_permutevar8x32_ps(signal_block_Q, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    cnt = 0;
    for(int i = 0; i < nof_inner_loop_iter; i++) {

      // Multiply eight elements of I.
      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_I, 0xff);
      dp_low_I = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);
      prod_I = _mm256_add_ps(dp_low_I, dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_Q, 0xff);
      dp_low_Q = _mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1);
      prod_Q = _mm256_add_ps(dp_low_Q, dp_high_Q);

      // Accumulates the result in the output vector.
      output[k*2*NOF_MULS_MM256 + cnt] += prod_I[0];
      output[k*2*NOF_MULS_MM256 + (cnt+1)] += prod_Q[0];
      cnt += 2;

      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+1], signal_block_I, 0xff);
      dp_low_I = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);
      prod_I = _mm256_add_ps(dp_low_I, dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+1], signal_block_Q, 0xff);
      dp_low_Q = _mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1);
      prod_Q = _mm256_add_ps(dp_low_Q, dp_high_Q);

      // Accumulates the result in the output vector.
      output[k*2*NOF_MULS_MM256 + cnt] += prod_I[0];
      output[k*2*NOF_MULS_MM256 + (cnt+1)] += prod_Q[0];
      cnt += 2;

      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+2], signal_block_I, 0xff);
      dp_low_I = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);
      prod_I = _mm256_add_ps(dp_low_I, dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+2], signal_block_Q, 0xff);
      dp_low_Q = _mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1);
      prod_Q = _mm256_add_ps(dp_low_Q, dp_high_Q);

      // Accumulates the result in the output vector.
      output[k*2*NOF_MULS_MM256 + cnt] += prod_I[0];
      output[k*2*NOF_MULS_MM256 + (cnt+1)] += prod_Q[0];
      cnt += 2;

      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+3], signal_block_I, 0xff);
      dp_low_I = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);
      prod_I = _mm256_add_ps(dp_low_I, dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+3], signal_block_Q, 0xff);
      dp_low_Q = _mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1);
      prod_Q = _mm256_add_ps(dp_low_Q, dp_high_Q);

      // Accumulates the result in the output vector.
      output[k*2*NOF_MULS_MM256 + cnt] += prod_I[0];
      output[k*2*NOF_MULS_MM256 + (cnt+1)] += prod_Q[0];
      cnt += 2;

      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+4], signal_block_I, 0xff);
      dp_low_I = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);
      prod_I = _mm256_add_ps(dp_low_I, dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+4], signal_block_Q, 0xff);
      dp_low_Q = _mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1);
      prod_Q = _mm256_add_ps(dp_low_Q, dp_high_Q);

      // Accumulates the result in the output vector.
      output[k*2*NOF_MULS_MM256 + cnt] += prod_I[0];
      output[k*2*NOF_MULS_MM256 + (cnt+1)] += prod_Q[0];
      cnt += 2;

      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+5], signal_block_I, 0xff);
      dp_low_I = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);
      prod_I = _mm256_add_ps(dp_low_I, dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+5], signal_block_Q, 0xff);
      dp_low_Q = _mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1);
      prod_Q = _mm256_add_ps(dp_low_Q, dp_high_Q);

      // Accumulates the result in the output vector.
      output[k*2*NOF_MULS_MM256 + cnt] += prod_I[0];
      output[k*2*NOF_MULS_MM256 + (cnt+1)] += prod_Q[0];
      cnt += 2;

      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+6], signal_block_I, 0xff);
      dp_low_I = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);
      prod_I = _mm256_add_ps(dp_low_I, dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+6], signal_block_Q, 0xff);
      dp_low_Q = _mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1);
      prod_Q = _mm256_add_ps(dp_low_Q, dp_high_Q);

      // Accumulates the result in the output vector.
      output[k*2*NOF_MULS_MM256 + cnt] += prod_I[0];
      output[k*2*NOF_MULS_MM256 + (cnt+1)] += prod_Q[0];
      cnt += 2;

      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+7], signal_block_I, 0xff);
      dp_low_I = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);
      prod_I = _mm256_add_ps(dp_low_I, dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i+7], signal_block_Q, 0xff);
      dp_low_Q = _mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1);
      prod_Q = _mm256_add_ps(dp_low_Q, dp_high_Q);

      // Accumulates the result in the output vector.
      output[k*2*NOF_MULS_MM256 + cnt] += prod_I[0];
      output[k*2*NOF_MULS_MM256 + (cnt+1)] += prod_Q[0];
      cnt += 2;
    }
  }
}

// input_length: Number of complex samples and not number of pairs of I and Q parts.
void execute_filter_mm256_v6(cf_t *input_complex, uint32_t input_length, cf_t *output_complex) {

  __m256 signal_block_0 __attribute__ ((aligned (32)));
  __m256 signal_block_1 __attribute__ ((aligned (32)));
  __m256 signal_block_I __attribute__ ((aligned (32)));
  __m256 signal_block_Q __attribute__ ((aligned (32)));
  __m256 dp_high_I __attribute__ ((aligned (32)));
  __m256 prod_I __attribute__ ((aligned (32)));
  __m256 dp_high_Q __attribute__ ((aligned (32)));
  __m256 prod_Q __attribute__ ((aligned (32)));

  __attribute__ ((aligned (32))) float *input = (float*)input_complex;
  __attribute__ ((aligned (32))) float *output = (float*)output_complex;

  int cnt;
  int nof_outer_loop_iter = input_length/NOF_MULS_MM256;

  for(int k = 0; k < nof_outer_loop_iter; k++) {

    signal_block_0 = _mm256_load_ps(input+(k*NOF_MULS_MM256*2));

    signal_block_1 = _mm256_load_ps(input+NOF_MULS_MM256+(k*NOF_MULS_MM256*2));

    signal_block_I = _mm256_permutevar8x32_ps(_mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(2, 0, 2, 0)), _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    signal_block_Q = _mm256_permutevar8x32_ps(_mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(3, 1, 3, 1)), _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    cnt = 0;
    for(int i = 0; i < (filter_mm256_context->filter_length + NOF_MULS_MM256 - 1); i++) {

      // Multiply eight elements of I.
      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_I, 0xff);

      prod_I = _mm256_add_ps(_mm256_permute2f128_ps(dp_high_I, dp_high_I, 1), dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_Q, 0xff);

      prod_Q = _mm256_add_ps(_mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1), dp_high_Q);

      // Accumulates the result in the output vector.
      output[k*2*NOF_MULS_MM256 + cnt] += prod_I[0];
      output[k*2*NOF_MULS_MM256 + (cnt+1)] += prod_Q[0];
      cnt += 2;
    }
  }
}

// input_length: Number of complex samples and not number of pairs of I and Q parts.
void execute_filter_mm256_v7(cf_t *input_complex, uint32_t input_length, cf_t *output_complex) {

  __m256 signal_block_0 __attribute__ ((aligned (32)));
  __m256 signal_block_1 __attribute__ ((aligned (32)));
  __m256 signal_block_I __attribute__ ((aligned (32)));
  __m256 signal_block_Q __attribute__ ((aligned (32)));

  __m256 res_I __attribute__ ((aligned (32)));
  __m256 res_Q __attribute__ ((aligned (32)));

  __attribute__ ((aligned (32))) float *input = (float*)input_complex;
  __attribute__ ((aligned (32))) float *output = (float*)output_complex;

  int cnt;
  int nof_outer_loop_iter = input_length/NOF_MULS_MM256;

  for(int k = 0; k < nof_outer_loop_iter; k++) {

    signal_block_0 = _mm256_load_ps(input+(k*NOF_MULS_MM256*2));

    signal_block_1 = _mm256_load_ps(input+NOF_MULS_MM256+(k*NOF_MULS_MM256*2));

    signal_block_I = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(2, 0, 2, 0));

    signal_block_I = _mm256_permutevar8x32_ps(signal_block_I, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    signal_block_Q = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(3, 1, 3, 1));

    signal_block_Q = _mm256_permutevar8x32_ps(signal_block_Q, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    cnt = 0;
    for(int i = 0; i < (filter_mm256_context->filter_length + NOF_MULS_MM256 - 1); i++) {

      res_I = _mm256_mul_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_I);

      res_I = _mm256_hadd_ps(res_I, res_I);

      res_I = _mm256_hadd_ps(res_I, res_I);

      res_I = _mm256_hadd_ps(res_I, res_I);

      res_Q = _mm256_mul_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_Q);

      res_Q = _mm256_hadd_ps(res_Q, res_Q);

      res_Q = _mm256_hadd_ps(res_Q, res_Q);

      res_Q = _mm256_hadd_ps(res_Q, res_Q);

      // Accumulates the result in the output vector.
      output[k*2*NOF_MULS_MM256 + cnt] += res_I[0];
      output[k*2*NOF_MULS_MM256 + (cnt+1)] += res_Q[0];
      cnt += 2;
    }
  }
}

// input_length: Number of complex samples and not number of pairs of I and Q parts.
void execute_filter_mm256_v8(cf_t *input_complex, uint32_t input_length, cf_t *output_complex) {

  __m256 signal_block_0 __attribute__ ((aligned (32)));
  __m256 signal_block_1 __attribute__ ((aligned (32)));
  __m256 signal_block_I __attribute__ ((aligned (32)));
  __m256 signal_block_Q __attribute__ ((aligned (32)));
  __m256 dp_I __attribute__ ((aligned (32)));
  __m256 dp_Q __attribute__ ((aligned (32)));
  __m256 perm_I __attribute__ ((aligned (32)));
  __m256 perm_Q __attribute__ ((aligned (32)));
  __m256 res_I __attribute__ ((aligned (32)));
  __m256 res_Q __attribute__ ((aligned (32)));

  __attribute__ ((aligned (32))) float *input = (float*)input_complex;
  __attribute__ ((aligned (32))) float *output = (float*)output_complex;

  int nof_samples = 2*NOF_MULS_MM256;
  int step = 0;
  int cnt;
  int nof_outer_loop_iter = input_length/NOF_MULS_MM256;
  int nof_inner_loop_iter = (filter_mm256_context->filter_length + NOF_MULS_MM256 - 1);

  for(int k = 0; k < nof_outer_loop_iter; k++) {

    signal_block_0 = _mm256_load_ps(input+(k*nof_samples));

    signal_block_1 = _mm256_load_ps(input+NOF_MULS_MM256+(k*nof_samples));

    signal_block_I = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(2, 0, 2, 0));

    signal_block_I = _mm256_permutevar8x32_ps(signal_block_I, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    signal_block_Q = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(3, 1, 3, 1));

    signal_block_Q = _mm256_permutevar8x32_ps(signal_block_Q, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

    cnt = 0;
    step = k*nof_samples;
    for(int i = 0; i < nof_inner_loop_iter; i++) {

      // Multiply eight elements of I.
      dp_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_I, 0xff);

      perm_I = _mm256_permute2f128_ps(dp_I, dp_I, 1);

      res_I = _mm256_add_ps(perm_I, dp_I);

      // Multiply eight elements of Q.
      dp_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_Q, 0xff);

      perm_Q = _mm256_permute2f128_ps(dp_Q, dp_Q, 1);

      res_Q = _mm256_add_ps(perm_Q, dp_Q);

      // Accumulates the result in the output vector.
      output[step + cnt] += res_I[0];
      output[step + cnt + 1] += res_Q[0];
      cnt += 2;
    }
  }
}

// input_length: Number of complex samples and not number of pairs of I and Q parts.
void execute_filter_mm256_v9(cf_t *input_complex, uint32_t input_length, cf_t *output_complex) {

  __m256 signal_block_0 __attribute__ ((aligned (32)));
  __m256 signal_block_1 __attribute__ ((aligned (32)));
  __m256 signal_block_I __attribute__ ((aligned (32)));
  __m256 signal_block_Q __attribute__ ((aligned (32)));
  __m256 dp_high_I __attribute__ ((aligned (32)));
  __m256 dp_low_I __attribute__ ((aligned (32)));
  __m256 prod_I __attribute__ ((aligned (32)));
  __m256 dp_high_Q __attribute__ ((aligned (32)));
  __m256 dp_low_Q __attribute__ ((aligned (32)));
  __m256 prod_Q __attribute__ ((aligned (32)));

  __m256i mask = _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7);

  const int imm8_I = _MM_SHUFFLE(2, 0, 2, 0);
  const int imm8_Q = _MM_SHUFFLE(3, 1, 3, 1);

  __attribute__ ((aligned (32))) float *input = (float*)input_complex;
  __attribute__ ((aligned (32))) float *output = (float*)output_complex;

  int cnt;
  int nof_outer_loop_iter = input_length/NOF_MULS_MM256;

  for(int k = 0; k < nof_outer_loop_iter; k++) {

    signal_block_0 = _mm256_load_ps(input+(k*NOF_SAMPLES_TO_READ));

    signal_block_1 = _mm256_load_ps(input+NOF_MULS_MM256+(k*NOF_SAMPLES_TO_READ));

    signal_block_I = _mm256_shuffle_ps(signal_block_0, signal_block_1, imm8_I);

    signal_block_I = _mm256_permutevar8x32_ps(signal_block_I, mask);

    signal_block_Q = _mm256_shuffle_ps(signal_block_0, signal_block_1, imm8_Q);

    signal_block_Q = _mm256_permutevar8x32_ps(signal_block_Q, mask);

    cnt = 0;
    for(int i = 0; i < (filter_mm256_context->filter_length + NOF_MULS_MM256 - 1); i++) {

      // Multiply eight elements of I.
      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_I, 0xff);

      dp_low_I = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);

      prod_I = _mm256_add_ps(dp_low_I, dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_Q, 0xff);

      dp_low_Q = _mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1);

      prod_Q = _mm256_add_ps(dp_low_Q, dp_high_Q);

      // Accumulates the result in the output vector.
      output[k*NOF_SAMPLES_TO_READ + cnt] += prod_I[0];
      output[k*NOF_SAMPLES_TO_READ + (cnt+1)] += prod_Q[0];
      cnt += 2;
    }
  }
}

// input_length: Number of complex samples and not number of pairs of I and Q parts.
void execute_filter_mm256_v10(cf_t *input_complex, uint32_t input_length, cf_t *output_complex) {

  __m256 signal_block_0 __attribute__ ((aligned (32)));
  __m256 signal_block_1 __attribute__ ((aligned (32)));
  __m256 signal_block_I __attribute__ ((aligned (32)));
  __m256 signal_block_Q __attribute__ ((aligned (32)));
  __m256 dp_high_I __attribute__ ((aligned (32)));
  __m256 dp_low_I __attribute__ ((aligned (32)));
  __m256 prod_I __attribute__ ((aligned (32)));
  __m256 dp_high_Q __attribute__ ((aligned (32)));
  __m256 dp_low_Q __attribute__ ((aligned (32)));
  __m256 prod_Q __attribute__ ((aligned (32)));

  __m256i mask = _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7);

  const int imm8_I = _MM_SHUFFLE(2, 0, 2, 0);
  const int imm8_Q = _MM_SHUFFLE(3, 1, 3, 1);

  __attribute__ ((aligned (32))) float *input = (float*)input_complex;
  __attribute__ ((aligned (32))) float *output = (float*)output_complex;

  int cnt = 0;
  int nof_outer_loop_iter = input_length/NOF_MULS_MM256;

  // Outer loop.
  for(int k = 0; k < nof_outer_loop_iter; k++) {

    signal_block_0 = _mm256_load_ps(input+(k*NOF_SAMPLES_TO_READ));

    signal_block_1 = _mm256_load_ps(input+(k*NOF_SAMPLES_TO_READ)+NOF_MULS_MM256);

    signal_block_I = _mm256_shuffle_ps(signal_block_0, signal_block_1, imm8_I);

    signal_block_I = _mm256_permutevar8x32_ps(signal_block_I, mask);

    signal_block_Q = _mm256_shuffle_ps(signal_block_0, signal_block_1, imm8_Q);

    signal_block_Q = _mm256_permutevar8x32_ps(signal_block_Q, mask);

    cnt = 0;
    // Inner loop.
    for(int i = 0; i < (filter_mm256_context->filter_length + NOF_MULS_MM256 - 1); i++) {

      // Multiply eight elements of I.
      dp_high_I = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_I, 0xff);

      dp_low_I = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);

      prod_I = _mm256_add_ps(dp_low_I, dp_high_I);

      // Multiply eight elements of Q.
      dp_high_Q = _mm256_dp_ps(filter_mm256_context->filter_mm256_kernel[i], signal_block_Q, 0xff);

      dp_low_Q = _mm256_permute2f128_ps(dp_high_Q, dp_high_Q, 1);

      prod_Q = _mm256_add_ps(dp_low_Q, dp_high_Q);

      // Accumulates the result in the output vector.
      output[k*NOF_SAMPLES_TO_READ + cnt] += prod_I[0];
      output[k*NOF_SAMPLES_TO_READ + cnt + 1] += prod_Q[0];
      cnt += 2;
    }
  }
}

void test_mm256_filter() {
  uint32_t filter_length = 5;
  initialize_filter_mm256(filter_coeffs_4, filter_length);

  uint32_t input_length = 9;

  int nof_additional_zeros = NOF_MULS_MM256 - (input_length % NOF_MULS_MM256);

  __attribute__ ((aligned (32))) cf_t input[input_length + nof_additional_zeros];
  bzero(input, sizeof(cf_t)*(input_length + nof_additional_zeros));
  input[0] = 1.0+0.0i;

  uint32_t output_length = input_length+filter_length+nof_additional_zeros-1;
  __attribute__ ((aligned (32))) cf_t output[output_length];
  bzero(output, sizeof(cf_t)*output_length);
}

void test_mm256_dot_product() {
  __m256 vec1 __attribute__ ((aligned (32))) = {1, 2, 3, 4, 5, 6, 7, 8};

  __m256 vec2 __attribute__ ((aligned (32))) = {1, 2, 3, 4, 5, 6, 7, 8};

  __m256 prod = _mm256_dp_ps(vec1, vec2, 0xff);

  float *v = NULL;

  v = (float*)&prod;

  for(int i = 0; i < 8; i++) {
    printf("%f - %f\n",v[i],prod[0]+prod[4]);
  }
}

void test_mm256_shuffle() {
  cf_t input_cf[] = {1.1+1.2i, 2.1+2.2i, 3.1+3.2i, 4.1+4.2i, 5.1+5.2i, 6.1+6.2i, 7.1+7.2i, 8.1+8.2i};

  float *input = (float*)input_cf;

  __m256 signal_block_0 __attribute__ ((aligned (32))) = _mm256_loadu_ps(input);

  __m256 signal_block_1 __attribute__ ((aligned (32))) = _mm256_loadu_ps(input+NOF_MULS_MM256);

  __m256 signal_block_I __attribute__ ((aligned (32))) = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(2, 0, 2, 0));

  signal_block_I = _mm256_permutevar8x32_ps(signal_block_I, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

  __m256 signal_block_Q __attribute__ ((aligned (32))) = _mm256_shuffle_ps(signal_block_0, signal_block_1, _MM_SHUFFLE(3, 1, 3, 1));

  signal_block_Q = _mm256_permutevar8x32_ps(signal_block_Q, _mm256_setr_epi32(0, 1, 4, 5, 2, 3, 6, 7));

  print_mm256_vector(&signal_block_0);
  print_mm256_vector(&signal_block_1);

  print_mm256_vector(&signal_block_I);
  print_mm256_vector(&signal_block_Q);
}

void test_mm256_permute() {
  __m256 vec1 __attribute__ ((aligned (32))) = {1, 2, 3, 4, 5, 6, 7, 8};

  __m256 vec2 __attribute__ ((aligned (32))) = {1, 2, 3, 4, 5, 6, 7, 8};

  __m256 dp_high_I __attribute__ ((aligned (32))) = _mm256_dp_ps(vec1, vec2, 0xff);

  __m256 dp_low_I __attribute__ ((aligned (32))) = _mm256_permute2f128_ps(dp_high_I, dp_high_I, 1);

  __m256 prod_I __attribute__ ((aligned (32))) = _mm256_add_ps(dp_low_I, dp_high_I);

  printf("%f - %f\n",prod_I[0],(dp_high_I[0]+dp_high_I[4]));
}

void test_filter_initialization() {
  uint32_t filter_length = 9;

  initialize_filter_mm256(filter_coeffs_8, filter_length);

  print_mm256_filter_kernel(&filter_mm256_context->filter_mm256_kernel[0], filter_length);
}

void print_mm256_vector(__m256 *prod) {
  float *v = (float*)prod;
  for(int i = 0; i < 8; i++) {
    printf("value[%d]: %f\n",i,v[i]);
  }
  printf("\n\n");
}

void print_mm256_filter_kernel(__m256 *prod, uint32_t filter_length) {
  for(int i = 0; i < (filter_length+NOF_MULS_MM256-1); i++) {
    printf("row[%d]:",i);
    float *v = (float*)&prod[i];
    for(int k = 0; k < NOF_MULS_MM256; k++) {
      printf(" %f",v[k]);
    }
    printf("\n");
  }
  printf("\n\n");
}

void print_filter_output(cf_t *output, uint32_t output_length) {
  for(int i = 0; i < output_length; i++) {
    printf("output[%d]: (%f,%f)\n",i, __real__ output[i], __imag__ output[i]);
  }
}

void test_mm256_filter_with_impulse() {
  uint32_t filter_length = 9;

  initialize_filter_mm256(filter_coeffs_8, filter_length);

  uint32_t input_length = 9;

  // Calculate the number of padding zeros so that input has a number of samples that is a multiple of 8.
  int nof_additional_zeros = NOF_MULS_MM256 - (input_length % NOF_MULS_MM256);

  __attribute__ ((aligned (32))) cf_t input[input_length + nof_additional_zeros];
  bzero(input, sizeof(cf_t)*(input_length + nof_additional_zeros));
  input[0] = 1.0+0.0i;

  uint32_t output_length = input_length+filter_length+nof_additional_zeros-1;
  __attribute__ ((aligned (32))) cf_t output[output_length];
  bzero(output, sizeof(cf_t)*output_length);

  execute_filter_mm256_v0(input, (input_length + nof_additional_zeros), output);

  print_filter_output(output, output_length);

  unitialize_filter_mm256();
}

void test_mm256_filter_profiling() {
  struct timespec filter_time;

  uint32_t input_length = 5760;

  uint32_t filter_length = 129;

  // Filter.
  __attribute__ ((aligned (32))) float filter_coeffs[filter_length];
  bzero(filter_coeffs, sizeof(float)*(filter_length));
  for(int i = 0; i < filter_length; i++) {
    float v = (float)rand()/(float)(RAND_MAX);
    filter_coeffs[i] = v;
  }

  // Input.
  __attribute__ ((aligned (32))) cf_t input[input_length];
  bzero(input, sizeof(cf_t)*(input_length));
  input[0] = 1.0+0.0i;

  // Output.
  uint32_t output_length = input_length+filter_length-1;
  __attribute__ ((aligned (32))) cf_t output[output_length];
  bzero(output, sizeof(cf_t)*output_length);

  // Intialize filter.
  initialize_filter_mm256(filter_coeffs, filter_length);

  int NUM_OF_ITER = 100000;
  double diff = 0;
  for(int nof_iter = 0; nof_iter < NUM_OF_ITER; nof_iter++) {

    // Timestamp start of filtering procedure.
    clock_gettime(CLOCK_REALTIME, &filter_time);

    //execute_filter_naive(input, input_length, filter_coeffs, filter_length, output);

    execute_filter_mm256_v8(input, input_length, output);

    diff += helpers_profiling_diff_time(filter_time);
  }
  printf("diff: %f\n",diff/NUM_OF_ITER);

  unitialize_filter_mm256();
}

void test_mm256_with_small_signal() {
  uint32_t filter_length = 9;

  initialize_filter_mm256(filter_coeffs_8, filter_length);

  uint32_t input_length = 9;

  // Calculate the number of padding zeros so that input has a number of samples that is a multiple of 8.
  int nof_additional_zeros = NOF_MULS_MM256 - (input_length % NOF_MULS_MM256);

  __attribute__ ((aligned (32))) cf_t input[16] = {
    -1.65528652385291 - 0.67838034748468400i, 2.159771846802570 - 0.391735130330852i, 0.315289654553879 + 1.08159370081825i, 0.431952586354754 + 1.323977679998490i,
    -1.32913667272087 - 0.00494513380786458i, 0.310348976076025 - 1.967136657417640i, 3.069366539866690 - 2.77884648104127i, 0.970015348093707 + 0.974690510951219i,
    0.222179708912873 - 0.33968481831363800i, 0.000000000000000 + 0.000000000000000i, 0.000000000000000 + 0.00000000000000i, 0.000000000000000 + 0.000000000000000i,
    0.000000000000000 + 0.00000000000000000i, 0.000000000000000 + 0.000000000000000i, 0.000000000000000 + 0.00000000000000i, 0.000000000000000 + 0.000000000000000i};

  uint32_t output_length = input_length+filter_length+nof_additional_zeros-1;
  __attribute__ ((aligned (32))) cf_t output[output_length];
  bzero(output, sizeof(cf_t)*output_length);

  execute_filter_mm256_v10(input, (input_length + nof_additional_zeros), output);

  print_filter_output(output, output_length);

  unitialize_filter_mm256();
}

void test_mm256_with_medium_signal() {
  uint32_t filter_length = 9;

  initialize_filter_mm256(filter_coeffs_8, filter_length);

  uint32_t input_length = 24;

  // Calculate the number of padding zeros so that input has a number of samples that is a multiple of 8.
  int nof_additional_zeros = 0;

  __attribute__ ((aligned (32))) cf_t input[24] = {
    -1.655286523853e+00 -1.225470465910e+00i, 2.159771846803e+00 +7.864017221184e-01i, 3.152896545539e-01 +8.694739135782e-01i, 4.319525863548e-01 +4.593794431638e-01i, -1.329136672721e+00 +1.849826424342e+00i, 3.103489760760e-01 +1.238119245241e+00i, 3.069366539867e+00 -1.511926334566e-01i, 9.700153480937e-01 +8.244194422819e-01i, 2.221797089129e-01 -8.129273004666e-01i, -6.783803474847e-01 -2.725480254977e-01i, -3.917351303309e-01 -9.992276695316e-01i, 1.081593700818e+00 -1.751469775937e+00i, 1.323977679998e+00 -2.708291888079e-01i, -4.945133807865e-03 -1.076011306978e+00i, -1.967136657418e+00 +1.569902315772e+00i, -2.778846481041e+00 +1.576522942374e+00i, 9.746905109512e-01 +2.806803546307e-01i, -3.396848183136e-01 +9.938710626243e-01i, 1.072552205631e-01 +5.066119389643e-01i, 3.005349681307e+00 -3.036482623757e-01i, -2.074139170969e-02 +1.431080410216e+00i, 7.109834008971e-02 +4.171864513965e-01i, -1.029349457643e+00 +3.508871875091e+00i, -3.761403961335e-01 +7.517340515830e-02i};

  uint32_t output_length = input_length+filter_length+nof_additional_zeros-1;
  __attribute__ ((aligned (32))) cf_t output[output_length];
  bzero(output, sizeof(cf_t)*output_length);

  execute_filter_mm256_v10(input, (input_length + nof_additional_zeros), output);

  print_filter_output(output, output_length);

  unitialize_filter_mm256();
}

int main() {

  test_mm256_with_medium_signal();

  return 0;
}
