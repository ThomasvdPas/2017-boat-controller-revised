#include "kalman.h"

//#include "arm_math.h"
//#include "math_helper.h"
#include "utilities/utilities.h"
#include "rtos.h"

#define kal_dt float32_t

#define kal_mat_inst  arm_matrix_instance_f32 
#define kal_mat_init(S, rows, cols, pdata) arm_mat_init_f32(S, rows, cols, pdata)
#define kal_mat_mul(SrcA, SrcB, Dst) arm_mat_mult_f32(SrcA, SrcB, Dst)
#define kal_mat_add(SrcA, SrcB, Dst) arm_mat_add_f32(SrcA, SrcB, Dst)
#define kal_mat_sub(SrcA, SrcB, Dst) arm_mat_sub_f32(SrcA, SrcB, Dst)
#define kal_mat_inv(Src, Dst) arm_mat_inverse_f32(Src, Dst)
#define kal_mat_trans(Src, Dst) arm_mat_trans_f32(Src, Dst)
extern Serial pc;
extern Mutex pcMutex;

/*
 * Matrices:
 * F (3x3) const 
 * B (3x2) const
 * u (2x1)
 * G (3x3) const 
 * H (3x3) const 
 * x (3x1)
 * u (2x1) XSens en versnellingscoef
 * Gamma (3x1) const
 * P (3x3)
 * Q (1x1) const
 * K (3x3)
 * Phi (3x3) const 
 * Psi (3x2) const
 * z (3x1) senix meting
 */


static kal_mat_inst xstate; //state vector of dim n
static kal_dt xstate_val[3] = {0.0,0,0}; //data array

static kal_mat_inst Pcov; //Covarance matrix
static kal_dt Pcov_val[9] = {10,10,10,10,10,10,10,10,10}; //array

static kal_mat_inst uinput; //input vector dim l
static kal_dt uinput_val[2] = {0.0, 0.0}; 

static kal_mat_inst zmeas; //measurement vector
static kal_dt zmeas_val[3]={0};

static kal_mat_inst K; //Kalman gain
static kal_dt K_val[3*3]={0};

static kal_mat_inst GammaQ; //Result of multiplication Gamma * Q * Gamma' (step 2)
static kal_dt GammaQ_val[3*3]={0}; //data array

static kal_mat_inst PhiT; //Phi transposed, cached in RAM
static kal_dt PhiT_val[3*3]={0}; //data array

static kal_mat_inst HT; //H transposed, cached in RAM
static kal_dt HT_val[3*3]={0};

static kal_dt Gamma_val[3] = {0.03, -00025, 0};//{-0.000003125, -0.0025, 0};
static kal_mat_inst Gamma = {3, 1 ,(kal_dt*)Gamma_val};

//Constant matrices
static const kal_dt H_val[9] = {1,0,0,0,0,0,0,0,0};
static const kal_mat_inst H = {3,3,(kal_dt*)H_val};

static const kal_dt R_val[3*3] = {0.0086, 0, 0, 0, 1, 0, 0, 0, 1};
static const kal_mat_inst R = {3,3,(kal_dt*)R_val};

static const kal_dt Q_val[1*1] = {0.000256};
static const kal_mat_inst Q = {1,1,(kal_dt*)Q_val};

static const kal_dt Psi_val[3*2] = {0.00005, -0.00005, 0.01, -0.01, 0, 0};
static const kal_mat_inst Psi = {3,2,(kal_dt*)Psi_val};

static const kal_dt Phi_val[3*3] = {1, 0.01, -0.00005, 0, 1, -0.01, 0, 0, 1};
static const kal_mat_inst Phi = {3,3,(kal_dt*)Phi_val};

static const kal_dt Eye_val[3*3] = 	{1,0,0,
					 0,1,0,
					 0,0,1}; //identity matrix
static const kal_mat_inst Eye = {3,3,(kal_dt*)Eye_val};








void kal_init(){
	//PRINT("1");
	kal_mat_init(&xstate, 3, 1, xstate_val);
	kal_mat_init(&Pcov, 3, 3, Pcov_val);
	kal_mat_init(&zmeas, 3, 1, zmeas_val);
	kal_mat_init(&K, 3, 3, K_val);
	kal_mat_init(&GammaQ, 3, 3, GammaQ_val);
	kal_mat_init(&uinput, 2, 1, uinput_val);
	kal_mat_init(&PhiT, 3,3, PhiT_val); 
	kal_mat_init(&HT, 3, 3, HT_val);
	kal_mat_init(&Gamma, 3, 1, Gamma_val);


	//Temporary matrixes

	//Calc GQG'
	kal_mat_inst tmp,tmp2;
	kal_dt tmp_val[3]={0};
	kal_dt tmp2_val[3]={0};
	kal_mat_init(&tmp, 3,1, tmp_val);
	kal_mat_init(&tmp2, 1,3, tmp2_val); // tmp2 = Gamma'

	kal_mat_mul(&Gamma, &Q, &tmp); //tmp = GammaQ
	kal_mat_trans(&Gamma, &tmp2);
	kal_mat_mul(&tmp, &tmp2, &GammaQ); //GammaQ = GammaQGamma'

	kal_mat_trans(&Phi, &PhiT);

	kal_mat_trans(&H, &HT);
	//PRINT("2");
}

void kal_prep(kal_dt xsensin, kal_dt senixin){
	uinput_val[0] = xsensin;
	zmeas_val[0] = senixin;
	//PRINT("x %f, s %f\n",xsensin, senixin);
}


/* Kalman filter step one:
 * x_kn+1 = Phi*x_kn + Psi*u
 *
 */
void kal_stepone(){
	kal_mat_inst tmp1_1,tmp1_2;

	kal_dt tmp_val[3]={0};
	//PRINT("%i, %i, %i \t", sizeof(kal_mat_inst), sizeof(kal_dt), sizeof(tmp_val));
	kal_mat_init(&zmeas, 3, 1, zmeas_val);
	kal_mat_init(&uinput, 2, 1, uinput_val);
	//x = Phi*x
	kal_mat_init(&tmp1_1, 3, 1, tmp_val);
	kal_mat_mul(&Phi, &xstate, &tmp1_1);
	// x+= Psi*u
	kal_dt tmp2_val[3]={0};
	kal_mat_init(&tmp1_2, 3, 1, tmp2_val);
	kal_mat_mul(&Psi, &uinput, &tmp1_2);
	kal_mat_add(&tmp1_1, &tmp1_2, &xstate);
	//PRINT("input: %f, %f \n", uinput.pData[0], uinput.pData[1]);
	//PRINT("States: %f, %f, %f \n", xstate.pData[0], xstate.pData[1], xstate.pData[2]);
}


/* Kalman filter step two:
 * Pcov_n+1 = Phi*Pcov_n*Phi' + G*Q*G'
 * G*Q*G result is cached
 *
 */
void kal_steptwo(){
	kal_mat_inst tmp2_1,tmp2_2;

	kal_dt tmp_val[9]={0};
	
	kal_mat_init(&tmp2_1, 3, 3, tmp_val);
	kal_mat_mul(&Phi, &Pcov, &tmp2_1); //tmp = Phi*Pcov
	// tmp2 = tmp*PhiT
	kal_dt tmp2_val[9]={0};
	kal_mat_init(&tmp2_2, 3,3, tmp2_val);
	kal_mat_mul(&tmp2_1, &PhiT, &tmp2_2);

	//Pcov = tmp2 + GQG'
	kal_mat_add(&tmp2_2, &GammaQ, &Pcov);
}



/* Kalman filter step three:
 * K = Pcov_n+1 * H' * (H*Pcov_n+1 *H' + R)^-1
 */
void kal_stepthree(){
	kal_mat_inst tmp3_1, tmp3_2, tmp3_3;
	kal_dt tmp1_val[9]={0};
	kal_dt tmp2_val[9]={0};
	kal_dt tmp3_val[9]={0};

	//Pcov_n+1 * H'
	kal_mat_init(&tmp3_1, 3, 3, tmp1_val);
	kal_mat_mul(&Pcov, &HT, &tmp3_1);

	kal_mat_init(&tmp3_2, 3, 3, tmp2_val);
	kal_mat_init(&tmp3_3, 3, 3, tmp3_val);

	//(H*Pcov_n+1 *H' + R)^-1
	kal_mat_mul(&H, &tmp3_1, &tmp3_2);
	kal_mat_add(&tmp3_2, &R, &tmp3_3);
	//PRINT("5");
	kal_mat_inv(&tmp3_3, &tmp3_2);
	//PRINT("6");
	
	//K = Pcov_n+1 * H' * (H*Pcov_n+1 *H' + R)^-1
	kal_mat_mul(&tmp3_1, &tmp3_2, &K);
	//PRINT("K= %f", K.pData[0]);
}


/* Kalman filter step four:
 * x_k+1n+1 = x_kn+1 + K * (Z- H * x_kn+1)
 */
void kal_stepfour(){
	//PRINT("8");
	kal_mat_inst tmp4_1,tmp4_2, tmp4_3, tmp4_4;
	kal_dt tmp_val[3]={0};
	kal_dt tmp2_val[3]={0};
	kal_dt tmp3_val[3]={0};
	kal_dt tmp4_val[3]={0};


	kal_mat_init(&tmp4_1, 3, 1, tmp_val);
	kal_mat_init(&tmp4_2, 3, 1, tmp2_val);
	kal_mat_mul(&H,&xstate, &tmp4_1);
	kal_mat_sub(&zmeas, &tmp4_1, &tmp4_2); //tmp2 = Z-H*x

	kal_mat_init(&tmp4_3, 3, 1, tmp3_val);
	kal_mat_mul(&K, &tmp4_2, &tmp4_3); //tmp = K * (Z-H*x)

	kal_mat_init(&tmp4_4, 3, 1, tmp4_val);
	kal_mat_add(&xstate, &tmp4_3, &tmp4_4); //tmp2 = xstate + K * (Z-H*x)

	//assign tmp2 to xstate
	memcpy(xstate.pData, tmp4_4.pData, sizeof(kal_dt)* xstate.numCols*xstate.numRows);

}


/* Kalman filter step five:
 * Pcov_n+1 = (I - K*H) * Pcov_n+1
 */
void kal_stepfive(){
	
	kal_mat_inst tmp5_1,tmp5_2;
	kal_dt tmp_val[9]={0};
	kal_dt tmp2_val[9]={0};

	kal_mat_init(&tmp5_1, 3, 3, tmp_val);
	kal_mat_init(&tmp5_2, 3, 3, tmp2_val);
	
	kal_mat_mul(&K, &H, &tmp5_1); //tmp = H*K
	kal_mat_sub(&Eye, &tmp5_1, &tmp5_2); //tmp2 = I-H*K

	kal_mat_init(&tmp5_1, 3, 3, tmp_val);
	kal_mat_mul(&tmp5_2, &Pcov, &tmp5_1); //tmp = (I-HK)*P

	//P = tmp
	memcpy(Pcov.pData, tmp5_1.pData, sizeof(kal_dt)* Pcov.numCols*Pcov.numRows);

}

float kalman_filter(float xsensin, float senixin){

	kal_prep(xsensin, senixin);
	kal_stepone();
	//PRINT("3");
	kal_steptwo();
	//PRINT("4");
	kal_stepthree();
	//PRINT("5");
	kal_stepfour();
	//PRINT("6");
	kal_stepfive();
	//PRINT("7");
	//PRINT("COV: %f, %f, %f", Pcov.pData[0], Pcov.pData[4], Pcov.pData[8]);
	return xstate.pData[0];
}
