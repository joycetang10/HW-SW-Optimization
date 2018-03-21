/* /////////////////////////////////////////////////////////////////////// */
/*  File   : find_motion.c                                                 */                                           
/*  Date   : 02/06/2017                                                    */
/* ----------------------------------------------------------------------- */
/*  This program will find the 16x16 block-based motion vectors between    */
/*  two 720x480 video frames in PGM format. Note that the PGM image format */
/*  is a subset of the PNM image format.                                   */
/* /////////////////////////////////////////////////////////////////////// */
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include "image.h"

#include "xparameters.h"  /* SDK generated parameters */
#include "xsdps.h"        /* for SD device driver     */
#include "ff.h"
#include "xil_cache.h"
#include "xplatform_info.h"
#include "xtime_l.h"

#include "xgpiops.h"
#define LED 7            /* The LED of PS7 on Zed connects to pin 7     */
XGpioPs Gpio;             /* Control structure for GPIO pins of Zynq PS7 */

/* Global Timer is always clocked at half of the CPU frequency */
#define COUNTS_PER_USECOND  (XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ / 2000000)
#define FREQ_MHZ ((XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ+500000)/1000000)
#define IP_SA 1
#define IP_CB 2

volatile int *R0 = (int *) (XPAR_ME_MATCH_0_S00_AXI_BASEADDR + 0);
volatile int *bank = (int *) (XPAR_ME_MATCH_0_S00_AXI_BASEADDR + 48);
volatile int *hw_active = (int *) (XPAR_ME_MATCH_0_S00_AXI_BASEADDR + 52);
volatile int *ip_mvx = (int *) (XPAR_ME_MATCH_0_S00_AXI_BASEADDR + 56);
volatile int *ip_mvy = (int *) (XPAR_ME_MATCH_0_S00_AXI_BASEADDR + 60);
volatile int *ip_sad = (int *) (XPAR_ME_MATCH_0_S00_AXI_BASEADDR + 64);
/* Declare a microsecond-resolution timer function */
long get_usec_time()
{
	XTime time_tick;

	XTime_GetTime(&time_tick);
	return (long) (time_tick / COUNTS_PER_USECOND);
}

#define BSIZE 16  /* Block size for motion estimation */
#define MSTEP  8  /* Step size between motion vectors */
typedef unsigned char uint8;
typedef struct {
    int8 x;
    int8 y;
} MVector;

/* function prototypes. */
void  median3x3(uint8 *image, int width, int height);
void  full_search(MVector *, uint8 *, uint8 *, int32, int32);
void  compute_statistics(float *, float *, float *, MVector *, int32);
void  print_motion_vectors(MVector *mv, int w, int h);
int block_bcopy(int ID, int bsize, uint8 *src, int src_w, int x, int y);
/* SD card I/O variables */
static FATFS fatfs;

int main(int argc, char **argv)
{
	XGpioPs_Config *gpio_cfg;
    CImage frame_1, frame_2;
    MVector *mv;
    int32 width, height, size;
    long tcount1, tcount2;
    float mean, min, max;

    /* Initialize the SD card driver. */
	if (f_mount(&fatfs, "0:/", 0))
	{
		return XST_FAILURE;
	}

    /* Initialize the Zynq PS7 GPIO pins */
    gpio_cfg = XGpioPs_LookupConfig(XPAR_PS7_GPIO_0_DEVICE_ID);
    if (XGpioPs_CfgInitialize(&Gpio, gpio_cfg, gpio_cfg->BaseAddr))
    {
        return XST_FAILURE;
    }
    XGpioPs_SetDirectionPin(&Gpio, LED, 1);
    XGpioPs_SetOutputEnablePin(&Gpio, LED, 1);

    /* Read image files into the DDR main memory */
    if (read_pnm_image("1.pgm", &frame_1))
    {
        printf("\nError: cannot read input image 1.\n");
    	return 1;
    }
    width = frame_1.width, height = frame_1.height;
    if (read_pnm_image("2.pgm", &frame_2))
    {
        printf("\nError: cannot read input image 2.\n");
        return 1;
    }
    if (width != frame_2.width || height != frame_2.height)
    {
        printf("\nError: Image sizes of the two frames do not match!\n");
        return 1;
    }

    /* Allocate space for storing motion vectors */
    size = (width/MSTEP)*(height/MSTEP);
    if ((mv = malloc(sizeof(MVector)*size)) == NULL)
    {
        printf("\nError: Fail to allocate memory for mvx[]!\n");
        return 1;
    }
    memset((char *) mv, 0, sizeof(MVector)*size);

    /* Turn on the LED to signal the start of computation. */
    XGpioPs_WritePin(&Gpio, LED, 0x1);
    printf("\nBegin motion estimation ...\n\n");

    /* Measuring computation time of median filtering. */
    tcount1 = get_usec_time();

    /* Perform median filter for noise removal */
    median3x3(frame_1.pix, width, height);
    median3x3(frame_2.pix, width, height);

    /* Measuring computation time of motion estimation. */
    tcount1 = (tcount2 = get_usec_time()) - tcount1;

    /* Perform full-search motion estimation */
    full_search(mv, frame_1.pix, frame_2.pix, width, height);

    /* End of computation. */
    tcount2 = get_usec_time() - tcount2;

    /* Turn off the LED to signal the end of computation. */
    XGpioPs_WritePin(&Gpio, LED, 0x0);

    /* Print the motion vector fields and some statistics of the vectors. */
    compute_statistics(&mean, &min, &max, mv, size);
    print_motion_vectors(mv, width/MSTEP, height/MSTEP);
    printf("The motion vectors have a mean of %4.1f pixels.\n", mean);
    printf("The motion vectors range between %4.1f and %4.1f pixels.\n", min, max);
    printf("It took %ld milliseconds to filter the two images.\n", tcount1/1000);
    printf("It took %ld milliseconds to estimate the motion field.\n", tcount2/1000);

    /* Free allocated memory */
    free(frame_1.pix);
    free(frame_2.pix);
    free(mv);

    return 0;
}

void matrix_to_array(uint8 *pix_array, uint8 *ptr, int width)
{
    int  idx, x, y;

    idx = 0;
    for (y = -1; y <= 1; y++)
    {
        for (x = -1; x <= 1; x++)
        {
            pix_array[idx++] = *(ptr+x+width*y);
        }
    }
}

void insertion_sort(uint8 *pix_array, int size)
{
    int idx, jdx;
    uint8 temp;

    for (idx = 1; idx < size; idx++)
    {
        for (jdx = idx; jdx > 0; jdx--)
        {
            if (pix_array[jdx] < pix_array[jdx-1])
            {
                /* swap */
                temp = pix_array[jdx];
                pix_array[jdx] = pix_array[jdx-1];
                pix_array[jdx-1] = temp;
            }
        }
    }
}
/*uint8 compare(const void *a, const void *b){
	return(*(uint8 *)a-*(uint8 *)b);
}
uint8 middle(uint8 *pix_array){
	//qsort(pix_array,9,sizeof(uint8),compare);
	uint8 max, min;
	if(pix_array[0] > pix_array[1] && pix_array[0] > pix_array[2] ) max = pix_array[0];
	if(pix_array[1] > pix_array[0] && pix_array[1] > pix_array[2]) max = pix_array[1];
	if(pix_array[2] > pix_array[0] && pix_array[2] > pix_array[1]) max = pix_array[2];

	if(pix_array[1] > pix_array[0] && pix_array[2] > pix_array[0] ) min = pix_array[0];
	if(pix_array[0] > pix_array[1] && pix_array[2] > pix_array[1]) min = pix_array[1];
	if(pix_array[1] > pix_array[2] && pix_array[1] > pix_array[2]) min = pix_array[2];

	return pix_array[0] ^ pix_array[1] ^ pix_array[2] ^ max ^ min;
}*/
void median3x3(uint8 *image, int width, int height)
{
    int   row, col;
    uint8 pix_array[9], *ptr;

    for (row = 1; row < height-1; row++)
    {
        for (col = 1; col < width-1; col++)
        {
            ptr = image + row*width + col;
            matrix_to_array(pix_array, ptr, width);
            //qsort(pix_array,9,sizeof(uint8),compare);
            insertion_sort(pix_array, 9);
            *ptr = pix_array[4];

        }
    }


}

int match(int *x, int *y, int posx, int posy, uint8 *prev, uint8 *curr, int width) {
	int min_sad;
	/* Copy CB and SA into the HW IP internal buffers */
	block_bcopy(IP_SA, 48, prev, width, posx-16, posy-16);
	block_bcopy(IP_CB, 16, curr, width, posx, posy);
	/* Now, trigger the IP to find x, y, and min_sad */
	*hw_active = 1;
	while (*hw_active);
	*x = *ip_mvx;
	*y = *ip_mvy;
	min_sad = *ip_sad;
	//printf("%d\n",min_sad);
	return min_sad;
}

int block_bcopy(int ID, int bsize, uint8 *src, int src_w, int x, int y) {
	int offset = y*src_w + x;
	if (ID != IP_SA && ID != IP_CB) return 1; /* Error! */
	*bank = (ID == IP_SA)? 0 : 48;
	for (int idx = 0; idx < bsize; idx++) {
		memmove((void *) R0, (void *) (src + offset), bsize);
		*bank += 1;
		offset += src_w;
	}
	return 0;
}

void full_search(MVector *mv, uint8 *prev_image, uint8 *curr_image, int32 width, int32 height)
/* Use full-search algorithm to find the motion vectors of the second */
/* image (curr_image) w.r.t. the first image (prev_image).            */
{
    int idx, idy, nx, ny;
    int x, y;

    /* Compute the number of movtion vectors per frame. */
    nx = width/MSTEP, ny = height/MSTEP;

    /* Although we declare mv[] as an 1D array, it is actually used as a 2D */
    /* array in row-major arrangement.  The width and height of mv[] are nx */
    /* and ny. For example, if the image size is 720x480, there are 45x30   */
    /* motion vectors.                                                      */

    /* Looping through the computation of the (nx-2)*(ny-2) motion vectors. */
    /* Note that we exclude the estimation of the vectors at the boundary   */
    /* positions to keep it simple. The boundary vectors are set to zero.   */
    for (idy = 2; idy < ny-4; idy++)
    {
        for (idx = 2; idx < nx-4; idx++)
        {
            /* Find the best match of the current block in the previous frame. */
            (void) match(&x, &y, (idx*MSTEP), (idy*MSTEP), prev_image, curr_image, width);

            /* Store the motion vector at the current position. */
            mv[idy*nx+idx].x = x, mv[idy*nx+idx].y = y;
        }
    }
}

void  print_motion_vectors(MVector *mv, int w, int h)
/* Print the motion vector field. */
{
    int idx, idy;
    char stemp[16];

    printf("\nThe motion vector field is as follows:\n\n");
    for (idy = 0; idy < h; idy++)
    {
        for (idx = 0; idx < w; idx++)
        {
            sprintf(stemp, "%d,%d", mv->x, mv->y);
            printf("%7s", stemp);
            mv++;
        }
        printf("\n");
    }
    printf("\n");
}

float quick_sqrt(float x)
{
    float xhalf = 0.5f*x;
    int i;

    memcpy((void *) &i, (void *) &x, sizeof(i)); // get bits for floating VALUE
    i = 0x5f375a86 - (i>>1); // gives initial guess y0
    memcpy((void *) &x, (void *) &i, sizeof(x)); // convert bits BACK to float
    x = x*(1.5f-xhalf*x*x);  // Newton step, repeating increases accuracy
    x = x*(1.5f-xhalf*x*x);  // Newton step, repeating increases accuracy
    x = x*(1.5f-xhalf*x*x);  // Newton step, repeating increases accuracy

    return 1/x;
}

void compute_statistics(float *mean, float *min, float *max, MVector *mv, int32 size)
{
    int idx;
    float sq, length, total;

    length = total = 0;
    *min = *max = 0;
    for (idx = 0; idx < size; idx++)
    {
        sq = (float) (mv[idx].x*mv[idx].x + mv[idx].y*mv[idx].y);
        length = quick_sqrt(sq);
        if (length < *min) *min = length;
        if (length > *max) *max = length;
        total += length;
    }
    *mean = total / size;
}
