#include "particle_filter.h"
#include "const_matrix.h"

using namespace std;
//
int main()
{
	bool start;
	data_t z_in[3];
	data_t sat_out[1];
	data_t x_out[STATE_DIM];
	bool read;
	bool write;
	static data_t z[3][TMAX];
	static data_t x[STATE_DIM][TMAX];
#pragma HLS ARRAY_PARTITION variable=z complete
#pragma HLS ARRAY_PARTITION variable=x complete



	static data_t r_out[STATE_DIM][N];
	static data_t r_out1[STATE_DIM][N];
#pragma HLS ARRAY_PARTITION variable=r_out complete
#pragma HLS ARRAY_PARTITION variable=r_out1 complete


	cout << "-----------------------------------------------" << endl;

	FILE *fp = fopen("r_matrix.bin", "rb");
	if (fp == NULL)printf("r_matrix.bin!\n");
	for (int idx1 = 0; idx1 < STATE_DIM; idx1++)
	{
		for (int idx2 = 0; idx2 < N; idx2++)
		{
#pragma HLS PIPELINE II= 5
			fread(&r_out1[idx1][idx2], sizeof(float), 1, fp);
		}
	}
	fclose(fp);
	//
	FILE *fp_z = fopen("z_matrix.bin", "rb");
	if (fp_z == NULL)printf("z_matrix.bin!\n");
	for (int idx1 = 0; idx1 < 3; idx1++)
	{
		for (int idx2 = 0; idx2 < TMAX; idx2++)
		{
#pragma HLS PIPELINE II= 5
			fread(&z[idx1][idx2], sizeof(float), 1, fp_z);
		}
	}
	fclose(fp_z);
	//
	FILE *fp_x = fopen("x_matrix.bin", "rb");
	if (fp_x == NULL)printf("x_matrix.bin!\n");
	for (int idx1 = 0; idx1 < STATE_DIM; idx1++)
	{
		for (int idx2 = 0; idx2 < TMAX; idx2++)
		{
#pragma HLS PIPELINE II= 5
			fread(&x[idx1][idx2], sizeof(float), 1, fp_x);
		}
	}
	fclose(fp_x);

	//
	int cnt_read = 0;
	int cnt_write = 0;
	start = 1;
	//

	while(sat_out[0] < 4)
	{
		if (read == 1)
		{
			z_in[0] = z[0][cnt_read];
			z_in[1] = z[1][cnt_read];
			z_in[2] = z[2][cnt_read];
			cnt_read ++;
		}

		particle_filter(start,z_in,sat_out,read,write,x_out);

		start = 0;
		if(write == 1)
		{
			cnt_write++;
			for (int idx1 = 0; idx1 < 6; idx1++)
			{
				data_t er = abs(x_out[idx1]-x[idx1][cnt_write]);
				cout << "er = " << er << endl;
			}
		}
	}
	//
	cout << "-----------------------------------------------" << endl;
	return 0;
}

