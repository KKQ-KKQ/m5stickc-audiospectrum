#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main(int argc, char **argv)
{
  int n_log2 = 10;
  int n, i;
  if (argc > 2) n_log2 = atoi(argv[1]);
  n = 1 << n_log2;
  
  printf("#define N_WAVE %d\n", n);
  printf("#define LOG2_N_WAVE %d\n", n_log2);
  printf("\n");
  printf("const short Fixed15FFT::Sinewave[Fixed15FFT::SAMPLE_SIZE] = {");
  
  for (i = 0; i < n - n/4; ++i) {
    double x = (double)i * (2*M_PI) / n;
    int y = sin(x) * 32767;
    if (i % 8 == 0) printf("\n   ");
    printf(" %6d,", y);
  }
  
  printf("\n");
  printf("};\n");
  printf("\n");

  printf("const short Fixed15FFT::Window[Fixed15FFT::N_WAVE] = {");
  for (i = 0; i < n; ++i) {
    double x = (double)i * (2*M_PI) / n;
    int y = (0.54 - 0.46 * cos(x)) * 32767;
    if (i % 8 == 0) printf("\n   ");
    printf(" %6d,", y);
  }
  
  printf("\n");
  printf("};\n");
  
  return 0;
}