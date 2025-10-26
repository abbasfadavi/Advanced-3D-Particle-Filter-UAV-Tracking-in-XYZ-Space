#include "particle_filter.h"
#include "const_matrix.h"
//
data_t custom_exp(data_t x)
{
#pragma HLS INLINE
	return hls::exp(x);
}
//
void particle_filter
(
		bool start,
		data_t z_in[3],
		data_t sat_out[1],
		bool &read,
		bool &write,
		data_t x_out[STATE_DIM]
)
{
	const data_t NN = 0.001;
	const data_t dt = 0.1;
	const data_t diag_q[STATE_DIM] = {0.1, 0.1, 0.1, 0.2236, 0.2236, 0.2236};

	static data_t sat = 0;
	static data_t p[STATE_DIM][N];
	static data_t x[STATE_DIM];
	static data_t process_noise[STATE_DIM][N];
	static data_t i = 1;
	static data_t innovation[3][N];
	static data_t mahalanobis[N];
	static data_t weights[N];
	static data_t weight_sum = 0;
	static data_t indices[N];
	static data_t CDF[N];
	static data_t u0 = 0;
	static data_t uu[N];
	static data_t inv_weight_sum = 0;
	static data_t dif[N];
	static data_t sum_dif = 0;
	static data_t sum_x;
	static data_t u1[N];

#pragma HLS INTERFACE ap_none port=start
#pragma HLS INTERFACE ap_none port=z_in
#pragma HLS INTERFACE ap_none port=sat_out
#pragma HLS INTERFACE ap_none port=x_out
#pragma HLS INTERFACE ap_none port=read
#pragma HLS INTERFACE ap_none port=write

#pragma HLS ARRAY_PARTITION variable=r complete
#pragma HLS ARRAY_PARTITION variable=p complete
#pragma HLS ARRAY_PARTITION variable= x_out complete
#pragma HLS ARRAY_PARTITION variable= z_in complete
	//
	read = 0;
	write = 0;
	if (start == 1)
	{
		//
#pragma HLS ARRAY_PARTITION variable=u1 complete
		INIT_U1: for (int idx = 0; idx < N; idx++)
		{
#pragma HLS UNROLL factor=8
			u1[idx] = idx * NN;
		}
		read= 1;
		sat = 1;
	}
	else if(sat == 1)
	{
		INIT_POS1: for (int idx1 = 0; idx1 < 3; idx1++)
			INIT_POS2: for (int idx2 = 0; idx2 < N; idx2++)
			{
#pragma HLS PIPELINE
				p[idx1][idx2] = z_in[idx1] + 1.41421356f * r[idx1][idx2];
			}

		INIT_VEL1 : for (int idx1 = 0; idx1 < 3; idx1++)
			INIT_VEL2 : for (int idx2 = 0; idx2 < N; idx2++)
			{
#pragma HLS PIPELINE
				p[idx1+3][idx2] = 0.5f * r[idx1][idx2];
			}

		sat = 2;
	}
	else if (sat == 2)
	{
		i = i + 1;

		PROCESS_NOISE1 : for (int idx1 = 0; idx1 < STATE_DIM; idx1++)
		{
			PROCESS_NOISE2 : for (int idx2 = 0; idx2 < N; idx2++)
			{
#pragma HLS PIPELINE II = 4
				process_noise[idx1][idx2] = diag_q[idx1] * r[idx1][idx2];
			}
		}

		UPDATE_POSITION1 : for (int idx1 = 0; idx1 < 3; idx1++)
			UPDATE_POSITION2 : for (int idx2 = 0; idx2 < N; idx2++)
			{
#pragma HLS PIPELINE II = 4
				p[idx1][idx2] = p[idx1][idx2] + dt * p[idx1+3][idx2];
			}

		ADD_NOISE1: for (int idx1 = 0; idx1 < STATE_DIM; idx1++)
			ADD_NOISE2 : for (int idx2 = 0; idx2 < N; idx2++)
			{
				p[idx1][idx2] = p[idx1][idx2] + process_noise[idx1][idx2];
			}

		read = 1;
		sat = 3;
	}
	else if (sat == 3)
	{
		CALC_INNOVATION1 : for (int idx1 = 0; idx1 < 3; idx1++)
			CALC_INNOVATION2 : for (int idx2 = 0; idx2 < N; idx2++)
			{
#pragma HLS PIPELINE
				innovation[idx1][idx2] = z_in[idx1] - p[idx1][idx2];
			}

		SQUARE_INNOVATION	 : for (int idx1 = 0; idx1 < 3; idx1++)
			SQUARE_INNOVATION2 : for (int idx2 = 0; idx2 < N; idx2++)
			{
				innovation[idx1][idx2] = innovation[idx1][idx2] * innovation[idx1][idx2];
			}

		loop1 : for (int idx1 = 0; idx1 < N; idx1++)
			innovation[2][idx1] = innovation[2][idx1] * 0.5f;

		loop2 : for (int idx2 = 0; idx2 < N; idx2++)
		{
#pragma HLS PIPELINE
			data_t sum = 0;
			loop3 : for (int idx1 = 0; idx1 < 3; idx1++)
			{
				sum += innovation[idx1][idx2];
			}
			mahalanobis[idx2] = -sum;
		}

		loop_exp :for (int idx1 = 0; idx1 < N; idx1++)
		{
#pragma HLS PIPELINE II = 3
			weights[idx1] = exp(mahalanobis[idx1]);
		}
		weight_sum = 0;
		for (int idx1 = 0; idx1 < N; idx1++)
			weight_sum = weight_sum + weights[idx1];
		inv_weight_sum = 1/weight_sum;

		NORMALIZE_WEIGHTS: for (int idx = 0; idx < N; idx++)
		{
#pragma HLS PIPELINE
			weights[idx] = weights[idx] * inv_weight_sum;
		}

		INIT_ARRAYS: for (int idx = 0; idx < N; idx++)
		{
#pragma HLS PIPELINE
			indices[idx] = 0;
			CDF[idx] = 0;
		}
		CDF[0] = weights[0];
		u0 = r[0][0] * NN;

		for (int idx = 1; idx < N; idx++)
			CDF[idx] = CDF[idx-1] + weights[idx];
		CALC_UU: for (int idx = 0; idx < N; idx++)
		{
#pragma HLS PIPELINE II=1
			uu[idx] = u0 + u1[idx];
		}
		CALC_DIF1 : for (int j = 0; j < N; j++)
		{
			CALC_DIF2 : for (int idx = 0; idx < N; idx++)
			{
#pragma HLS PIPELINE II=1
				dif[idx] = (uu[j] > CDF[idx]) ? 1.0f : 0.0f;
			}
			sum_dif = 0;
			CALC_DIF3 : for (int k = 0; k < N; k++)
			{
#pragma HLS PIPELINE II=1
				sum_dif = sum_dif + dif[k];
			}
			sum_dif = sum_dif + 1;
			if (sum_dif > N)
			{
				sum_dif = N;
			}
			indices[j] = sum_dif;
		}

		data_t p_temp[STATE_DIM][N];
#pragma HLS ARRAY_PARTITION variable=p_temp complete dim=1
		RESAMPLE: for (int idx2 = 0; idx2 < N; idx2++)
		{
#pragma HLS PIPELINE
			for (int idx1 = 0; idx1 < STATE_DIM; idx1++)
			{
#pragma HLS UNROLL
				p_temp[idx1][idx2] = p[idx1][idx2];
			}
		}

		COPY_BACK: for (int idx1 = 0; idx1 < STATE_DIM; idx1++)
		{
#pragma HLS UNROLL
			for (int idx2 = 0; idx2 < N; idx2++)
			{
#pragma HLS PIPELINE II=1

				int ind = indices[idx2] - 1;
				p[idx1][idx2] = p_temp[idx1][ind];
			}
		}

		ACCUMULATE_X2 : for (int idx1 = 0; idx1 < STATE_DIM; idx1++)
		{
			sum_x = 0;
			ACCUMULATE_X1 : for (int idx2 = 0; idx2 < N; idx2++)
			{
#pragma HLS PIPELINE II=1
				sum_x = sum_x + p[idx1][idx2];
			}
			x[idx1] = sum_x* NN;
		}
		//
		write = 1;
		//
		if (i == TMAX)sat = 4;
		else sat = 2;
	}

	for (int idx = 0; idx < STATE_DIM; idx++)
	{
#pragma HLS UNROLL
		x_out[idx] = x[idx];
	}
	sat_out[0] = sat;
}
