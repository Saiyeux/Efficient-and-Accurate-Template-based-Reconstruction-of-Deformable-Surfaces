#include "objReader.h"
#include "helper.h"
// Include Libraries
#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <map>
#include <Eigen/Core>
#include "suitesparse/cholmod.h"

// Namespace to nullify use of cv::function(); syntax
using namespace std;
using namespace cv;

//Solve Sparse Matrix using CHOLMOD (http://www.cise.ufl.edu/research/sparse/SuiteSparse/) 
cholmod_sparse *m_cholSparseS;				
cholmod_factor *m_cholFactorS; 
cholmod_common m_cS; 
cholmod_dense  *m_cholSparseR, *m_cholSparseE;

//use sba_crsm structure from SBA (http://www.ics.forth.gr/~lourakis/sba/) to store sparse matrix
struct sba_crsm	
{
    int nr, nc;   
    int nnz;      
    int *val;     
    int *colidx;  
    int *rowptr;  
};

//cite from SBA code in order to search sub matrix of S according to (i,j)
static void sba_crsm_alloc(struct sba_crsm *sm, int nr, int nc, int nnz)
{
	int msz;
	sm->nr=nr;
	sm->nc=nc;
	sm->nnz=nnz;
	msz=2*nnz+nr+1;
	sm->val=(int *)malloc(msz*sizeof(int));  /* required memory is allocated in a single step */
	if(!sm->val){
		fprintf(stderr, "memory allocation request failed in sba_crsm_alloc() [nr=%d, nc=%d, nnz=%d]\n", nr, nc, nnz);
		exit(1);
	}
	sm->colidx=sm->val+nnz;
	sm->rowptr=sm->colidx+nnz;
}

static void sba_crsm_free(struct sba_crsm *sm)
{
	 sm->nr=sm->nc=sm->nnz=-1;
	free(sm->val);
	sm->val=sm->colidx=sm->rowptr=NULL;
}

/* returns the index of the (i, j) element. No bounds checking! */ // from PBA
static int sba_crsm_elmidx(struct sba_crsm *sm, int i, int j)
{
	int low, high, mid, diff;

	low=sm->rowptr[i];
	high=sm->rowptr[i+1]-1;

	/* binary search for finding the element at column j */
	while(low<=high)
	{
		mid=(low+high)>>1; //(low+high)/2;
		diff=j-sm->colidx[mid];
		if(diff<0)
			 high=mid-1;
		else if(diff>0)
			low=mid+1;
		else
		return mid;
	}

	return -1; /* not found */
}

void constructSmask( sba_crsm& Sidxij, int m, int& m_nS, char* m_smask)//, sba_crsm& Uidxij, char* m_umask)
{
	int i, j, k, ii, jj;
    for ( i = 0; i < m*m; i++ )
	{
		if ( (m_smask[i] == 1) )
		{
            
			m_smask[i] = 1;
			m_nS += 1;
		}
	}	
	sba_crsm_alloc(&Sidxij, m, m, m_nS);
	for(i=k=0; i<m; ++i)
	{
		Sidxij.rowptr[i]=k;
		ii=i*m;
		for(j=0; j<m; ++j)
			if(m_smask[ii+j])
			{
				Sidxij.val[k]=k;
				Sidxij.colidx[k++]=j;
			}
	}
	Sidxij.rowptr[m]=m_nS;
    
    // jj=m*m;
    // nuis = 0;
    // for(i=0; i<jj; i++)
	// 	nuis+=(m_umask[i]==1);
    // // std::cout << nuis << std::endl; 
    // // exit(1);
	// sba_crsm_alloc(&Uidxij, m, m, nuis);
	// for(i=k=0; i<m; ++i)
	// {
	// 	Uidxij.rowptr[i]=k;
	// 	ii=i*m;
	// 	for(j=0; j<m; ++j)
	// 		if(m_umask[ii+j])
	// 		{
	// 			Uidxij.val[k]=k;
	// 			Uidxij.colidx[k++]=j;
	// 		}
	// }
	// Uidxij.rowptr[m]=nuis;
	
	// return nuis;
}



void compute_reprojection_error(double *e, double *obs, double *xyz, int *faces, Eigen::Matrix3d &K, int num_obs) {
    for(int obs_id=0; obs_id < num_obs; obs_id++) {
        double fx = K(0,0);
        double fy = K(1,1);
        double cx = K(0,2);
        double cy = K(1,2);

        int face_id = obs[obs_id * 6];
        int f1 = faces[face_id*3];
        int f2 = faces[face_id*3+1];
        int f3 = faces[face_id*3+2];

        // double v1[] = {xyz[f1*3],xyz[f1*3+1],xyz[f1*3+2]};
        double *v1= &xyz[f1*3];
        // double v2 = {xyz[f2*3],xyz[f2*3+1],xyz[f2*3+2]};
        double *v2 = &xyz[f2*3];
        // double v3 = {xyz[f3*3],xyz[f3*3+1],xyz[f3*3+2]};
        double *v3 = &xyz[f3*3];

        double alpha = obs[obs_id * 6 + 3];
        double beta = obs[obs_id * 6 + 4];
        double gamma = obs[obs_id * 6 + 5];

        double z_u = obs[obs_id * 6 + 1];
        double z_v = obs[obs_id * 6 + 2];

        //  std::cout << v1[0] << " " << v1[1] << " " << v1[2] << std::endl;
        
        double p[3] = {v1[0] * alpha + v2[0]*beta + v3[0]*gamma,
                       v1[1] * alpha + v2[1]*beta + v3[1]*gamma,
                       v1[2] * alpha + v2[2]*beta + v3[2]*gamma};

        double a = fx * p[0] / p[2] + cx;
        double b = fy * p[1] / p[2] + cy;
        double e_u = z_u - (fx * p[0] / p[2] + cx);
        double e_v = z_v - (fy * p[1] / p[2] + cy);
        
        e[obs_id*2 + 0] = e_u;
        e[obs_id*2 + 1] = e_v;
    }
}

inline void storeInG_distance(double *g, double *J, double *e) {
    g[0] += -J[0]*e[0];
    g[1] += -J[1]*e[0];
    g[2] += -J[2]*e[0];

}

inline void storeInV_distance(double* V, int idx1, int idx2, double* J1, double* J2, sba_crsm& Uidxij) {
    int pos;
    double* ppUpa;
    double *JJ1, *JJ2;
    double sum=0;	
    if (idx1 < idx2) {
        JJ1 = J1;
        JJ2 = J2;
        pos = sba_crsm_elmidx( &Uidxij, idx1, idx2);
    } else {
        JJ1 = J2;
        JJ2 = J1;
        pos = sba_crsm_elmidx( &Uidxij, idx2, idx1); 
    }

    if(pos == -1) {
        std::cerr << "Pos -1 in StoreInV" << std::endl; 
    }
    ppUpa = V + pos * (3*3);
    ppUpa[0] += JJ1[0] * JJ2[0];
    ppUpa[1] += JJ1[0] * JJ2[1];
    ppUpa[2] += JJ1[0] * JJ2[2];
    ppUpa[3] += JJ1[1] * JJ2[0];
    ppUpa[4] += JJ1[1] * JJ2[1];
    ppUpa[5] += JJ1[1] * JJ2[2];
    ppUpa[6] += JJ1[2] * JJ2[0];
    ppUpa[7] += JJ1[2] * JJ2[1];
    ppUpa[8] += JJ1[2] * JJ2[2];
    
}

inline void storeInV_distance(double* V, int idx, double* J, sba_crsm& Uidxij) {
    int pos;
    double* ppUpa;
    double sum=0;	
    pos = sba_crsm_elmidx( &Uidxij, idx, idx);

    if(pos == -1) {
        std::cerr << "Pos -1 in StoreInV" << std::endl; 
    }
    ppUpa = V + pos * (3*3);
    ppUpa[0] += J[0] * J[0];
    ppUpa[1] += J[0] * J[1];
    ppUpa[2] += J[0] * J[2];

    ppUpa[4] += J[1] * J[1];
    ppUpa[5] += J[1] * J[2];

    ppUpa[8] += J[2] * J[2];
}

void compute_distance_jacobian(double *V, double *g, double *error, double *xyz, int *faces, int num_faces, int offset, sba_crsm& Uidxij) {
    int counter = 0;
    for(int i=0;i < num_faces; i++) {
        
        int f1 = faces[i*3];
        int f2 = faces[i*3+1];
        int f3 = faces[i*3+2];

        Eigen::Vector3d v1, v2, v3, v12, v13, v23;
        v1 << xyz[f1*3], xyz[f1*3+1], xyz[f1*3+2];
        v2 << xyz[f2*3], xyz[f2*3+1], xyz[f2*3+2];
        v3 << xyz[f3*3], xyz[f3*3+1], xyz[f3*3+2];

        double d12, d13, d23;
        v12 = v1-v2;
        d12 = v12.norm();
        v13 = v1-v3;
        d13 = v13.norm();
        v23 = v2 - v3;
        d23 = v23.norm();

        double J12_1[3], J12_2[3], J13_1[3], J13_3[3], J23_2[3], J23_3[3];

        J12_1[0] = v12[0] / (d12 + 0.0000001);
        J12_1[1] = v12[1] / (d12 + 0.0000001);
        J12_1[2] = v12[2] / (d12 + 0.0000001);

        J13_1[0] = v13[0] / (d13 + 0.0000001);
        J13_1[1] = v13[1] / (d13 + 0.0000001);
        J13_1[2] = v13[2] / (d13 + 0.0000001);

        J23_2[0] = v23[0] / (d23 + 0.0000001);
        J23_2[1] = v23[1] / (d23 + 0.0000001);
        J23_2[2] = v23[2] / (d23 + 0.0000001);

        J12_2[0] = -v12[0] / (d12 + 0.0000001);
        J12_2[1] = -v12[1] / (d12 + 0.0000001);
        J12_2[2] = -v12[2] / (d12 + 0.0000001);

        J13_3[0] = -v13[0] / (d13 + 0.0000001);
        J13_3[1] = -v13[1] / (d13 + 0.0000001);
        J13_3[2] = -v13[2] / (d13 + 0.0000001);

        J23_3[0] = -v23[0] / (d23 + 0.0000001);
        J23_3[1] = -v23[1] / (d23 + 0.0000001);
        J23_3[2] = -v23[2] / (d23 + 0.0000001);

        storeInG_distance(g+f1*3, J12_1, error + offset*2 + counter);
        storeInG_distance(g+f2*3, J12_2, error + offset*2 + counter);
        counter++;
        storeInG_distance(g+f1*3, J13_1, error + offset*2 + counter);
        storeInG_distance(g+f3*3, J13_3, error + offset*2 + counter);
        counter++;
        storeInG_distance(g+f2*3, J23_2, error + offset*2 + counter);
        storeInG_distance(g+f3*3, J23_3, error + offset*2 + counter);
        counter++;
        // storeInG(g+f1*3, J1, error+obs_id);

        storeInV_distance(V, f1, J12_1, Uidxij);
        storeInV_distance(V, f2, J12_2, Uidxij);

        storeInV_distance(V, f1, J13_1, Uidxij);
        storeInV_distance(V, f3, J13_3, Uidxij);

        storeInV_distance(V, f2, J23_2, Uidxij);
        storeInV_distance(V, f3, J23_3, Uidxij);

        storeInV_distance(V, f1, f2, J12_1, J12_2, Uidxij);
        storeInV_distance(V, f1, f3, J13_1, J13_3, Uidxij);
        storeInV_distance(V, f2, f3, J23_2, J23_3, Uidxij);
    }
    // std::cout << "dsad";
}


void compute_distance_error(double *error, double *xyz, double *ref,int *faces, int number_faces, int offset) {
    int counter = 0;
    for(int i=0;i<number_faces; i++) {
        
        int f1 = faces[i*3];
        int f2 = faces[i*3+1];
        int f3 = faces[i*3+2];

        Eigen::Vector3d v1, v2, v3, v1_ref, v2_ref, v3_ref;
        v1 << xyz[f1*3], xyz[f1*3+1], xyz[f1*3+2];
        v2 << xyz[f2*3], xyz[f2*3+1], xyz[f2*3+2];
        v3 << xyz[f3*3], xyz[f3*3+1], xyz[f3*3+2];

        v1_ref << ref[f1*3], ref[f1*3+1], ref[f1*3+2];
        v2_ref << ref[f2*3], ref[f2*3+1], ref[f2*3+2];
        v3_ref << ref[f3*3], ref[f3*3+1], ref[f3*3+2];

        double d12, d12_ref, d13, d13_ref, d23, d23_ref;
        d12 = (v1-v2).norm();
        d12_ref = (v1_ref-v2_ref).norm();

        d13 = (v1-v3).norm();
        d13_ref = (v1_ref-v3_ref).norm();

        d23 = (v2-v3).norm();
        d23_ref = (v2_ref-v3_ref).norm();
        double e1 = d12 - d12_ref;
        double e2 = d13 - d13_ref;
        double e3 = d23 - d23_ref;
        error[offset*2 + counter] = e1;
        counter++;
        error[offset*2 + counter] = e2;
        counter++;
        error[offset*2 + counter] = e3;
        counter++;
    }
}

inline void storeInG(double *g, double *J, double *e) {
    g[0] += -(J[0] * e[0] + J[3] * e[1]);
    g[1] += -(J[1] * e[0] + J[4] * e[1]);
    g[2] += -(J[2] * e[0] + J[5] * e[1]);
    // cout << g[0] << " " << g[1] << " "<< g[2] << endl;
    // cout << "" <<endl;
}

void storeInV(double* V, int idx, double* J, sba_crsm& Uidxij) {
    int pos;
    double* ppUpa;
    double sum=0;	
    pos = sba_crsm_elmidx( &Uidxij, idx, idx);
    
    if(pos == -1) {
        std::cerr << "Pos -1 in StoreInV" << std::endl; 
    }
    ppUpa = V + pos * (3*3);

    for(int i=0; i < 3; i++) {
        for(int j=i; j < 3; j++) {
            sum = 0;
            for ( int k = 0; k < 2; k++ ) {
				sum += J[i+k*3]*J[j+k*3];
            }
            ppUpa[i*3+j] += sum; 
            // if( i<=j)
            //     std::cout << sum << "\t\t";
            // else
            //     std::cout << 0 << "\t\t";
        }
        // cout << "\n";
    }
    // exit(1);
}

void storeInV(double* V, int idx1, int idx2, double* J1, double* J2, sba_crsm& Uidxij) {
    int pos;
    double* ppUpa;
    double *JJ1, *JJ2;
    double sum=0;	
    if (idx1 < idx2) {
        JJ1 = J1;
        JJ2 = J2;
        pos = sba_crsm_elmidx( &Uidxij, idx1, idx2);
    } else {
        JJ1 = J2;
        JJ2 = J1;
        pos = sba_crsm_elmidx( &Uidxij, idx2, idx1); 
    }
    if(pos == -1) {
        std::cerr << "Pos -1 in StoreInV" << std::endl; 
    }
    ppUpa = V + pos * (3*3);
    for(int i=0; i < 3; i++) {
        for(int j=0; j < 3; j++) {
            sum = 0;
            for ( int k = 0; k < 2; k++ ) {
				sum += JJ1[i+k*3]*JJ2[j+k*3];
            }
            ppUpa[i*3+j] += sum; // Vill noch anpassen
        }
    }
}

void compute_reprojection_jacobian(double *V, double *g, double *error, double *obs, double *xyz, int *faces, Eigen::Matrix3d K, int num_obs, sba_crsm& Sidxij) {
    for(int obs_id=0; obs_id < num_obs; obs_id++) {
        double fx = K(0,0);
        double fy = K(1,1);
        double cx = K(0,2);
        double cy = K(1,2);

        int face_id = obs[obs_id * 6];
        int f1 = faces[face_id*3];
        int f2 = faces[face_id*3+1];
        int f3 = faces[face_id*3+2];

        double *v1= &xyz[f1*3];
        double *v2 = &xyz[f2*3];
        double *v3 = &xyz[f3*3];

        double alpha = obs[obs_id * 6 + 3];
        double beta = obs[obs_id * 6 + 4];
        double gamma = obs[obs_id * 6 + 5];

        double x = v1[0] * alpha + v2[0]*beta + v3[0]*gamma;
        double y = v1[1] * alpha + v2[1]*beta + v3[1]*gamma;
        double z = v1[2] * alpha + v2[2]*beta + v3[2]*gamma;

        double J1[6], J2[6], J3[6];
        J1[0] = -fx/z * alpha;
        J1[1] = 0;
        J1[2] = fx*x/(z*z) * alpha;

        J1[3] = 0;
        J1[4] = -fy/z * alpha;
        J1[5] = fy*y/(z*z) * alpha;

        J2[0] = -fx/z * beta;
        J2[1] = 0;
        J2[2] = fx*x/(z*z) * beta;

        J2[3] = 0;
        J2[4] = -fy/z * beta;
        J2[5] = fy*y/(z*z) * beta;

        J3[0] = -fx/z * gamma;
        J3[1] = 0;
        J3[2] = fx*x/(z*z) * gamma;

        J3[3] = 0;
        J3[4] = -fy/z * gamma;
        J3[5] = fy*y/(z*z) * gamma;

        storeInG(g+f1*3, J1, error+obs_id * 2);
        storeInG(g+f2*3, J2, error+obs_id * 2);
        storeInG(g+f3*3, J3, error+obs_id * 2);

        storeInV(V, f1, J1, Sidxij);
        storeInV(V, f2, J2, Sidxij);
        storeInV(V, f3, J3, Sidxij);

        // if(f1==1663 || f2==1663 || f3==1663 ){
        //     cout << "asdssda\n\n";
        // }        

        storeInV(V, f1, f2, J1, J2, Sidxij);
        storeInV(V, f1, f3, J1, J3, Sidxij);
        storeInV(V, f2, f3, J2, J3, Sidxij);
    }
}

void constructAuxCSSGN( int *Ap, int *Aii, char* m_smask, int m_ncams)
{
	int* Cp = Ap;
	int* Ci = Aii;
	int ii, jj;
	int m = m_ncams, nZ = 0;
	for ( ii = 0; ii < m; ii++ ) 
	{
		*Cp = nZ;
		for( jj=0; jj<=ii; jj++ )
		{
			if (m_smask[jj*m+ii]==1)
			{
				*Ci++ = jj;
				nZ++;
			}
		}
		Cp++;
	}
	*Cp=nZ;
}

void constructCSSGN( int* Si, int* Sp, double* Sx, double* S, sba_crsm& Sidxij, bool init, char* m_smask, int m_ncams)
{
	int ii, jj, jjj, k;
	int pos1, m = m_ncams;
	//Copy S matrix and E matrix to specific format structure for Cholmod 
	double *ptr5;
	int nZ = 0;
	Sx = (double*)m_cholSparseS->x;

	if ( !init)
	{
		for ( ii = 0; ii < m; ii++ )  //column
		{
			for ( k = 0; k < 3; k++ )
			{
				*Sp = nZ;
				// if ((ii*6+k)==(9+nft))
				// 	continue;

				for ( jj = 0; jj <= ii; jj++ )	//row
				{   
                    // std::cout << (int)m_smask[ii+jj*m] << "\t";
					if ((m_smask[jj*m+ii]==1))
					{   

						pos1 = sba_crsm_elmidx( &Sidxij, jj, ii );
						ptr5 = S + pos1*9;
						
						if( ii == jj )
						{
							for ( jjj = 0; jjj <= k; jjj++)
							{
								// if ( (jj*6+jjj) != (9+nft))
								// {
									// if ( jj*6+jjj < 9+nft)
									// 	*Si++ = jj*6+jjj - 6;
									// else
									// 	*Si++ = jj*6+jjj - 7;
                                    *Si++ = jj*3+jjj;
									*Sx++ = ptr5[jjj*3+k];
									nZ++;
                                    // std::cout << ptr5[jjj*6+k] << std::endl;
								// }						
							}
                            // exit(1);
						}
						else
						{
							for ( jjj = 0; jjj < 3; jjj++)
							{
								// if ((jj*6+jjj) != (9+nft) )
								// {
									// if ( jj*6+jjj < 9+nft )
									// 	*Si++ = jj*6+jjj - 6;
									// else
									// 	*Si++ = jj*6+jjj - 7;										

                                    *Si++ = jj*3+jjj;
									*Sx++ = ptr5[jjj*3+k];
									nZ++;
								// }
							}
						}
					}
				} 
                // std::cout << "\n";
				Sp++;
			}
		}
		*Sp=nZ;
	}
	else
	{
        // std::cout << "else" << std::endl;
		for ( ii = 0; ii < m; ii++ )  //column
		{
			for ( k = 0; k < 3; k++ )
			{
				// if ((ii*6+k)==(9+nft))
				// 	continue;

				for ( jj = 0; jj <= ii; jj++ )	//row
				{
					if ((m_smask[jj*m+ii]==1))
					{
						pos1 = sba_crsm_elmidx( &Sidxij, jj, ii );
						ptr5 = S + pos1*9;

						if( ii == jj )
						{
							for ( jjj = 0; jjj <= k; jjj++)
							{
								// if ( (jj*6+jjj) != (9+nft))
									*Sx++ = ptr5[jjj*3+k];
							}
						}
						else
						{
							for ( jjj = 0; jjj < 3; jjj++)
							{
								// if ((jj*6+jjj) != (9+nft) )
									*Sx++ = ptr5[jjj*3+k];
							}
						}
					}
				}
			}
		}
	}
}

bool solveCholmodGN( int* Ap, int* Aii, bool init, bool ordering, int m_ncams, int nnz)
{
	int i, j;
	int m = m_ncams;
	Eigen::VectorXi scalarPermutation, blockPermutation;

	ordering = true;
	if (!init)
	{
		if (!ordering)
		{
			m_cS.nmethods = 1;
			m_cS.method[0].ordering = CHOLMOD_AMD; //CHOLMOD_COLAMD
			m_cholFactorS = cholmod_analyze(m_cholSparseS, &m_cS); // symbolic factorization
		}
		else
		{

			// get the ordering for the block matrix
			if (blockPermutation.size() == 0)
				blockPermutation.resize(m_ncams);
            
			// prepare AMD call via CHOLMOD
			cholmod_sparse auxCholmodSparse;
			auxCholmodSparse.nzmax = nnz;
			auxCholmodSparse.nrow = auxCholmodSparse.ncol = m;
			auxCholmodSparse.p = Ap;
			auxCholmodSparse.i = Aii;
			auxCholmodSparse.nz = 0;
			auxCholmodSparse.x = 0;
			auxCholmodSparse.z = 0;
			auxCholmodSparse.stype = 1;
			auxCholmodSparse.xtype = CHOLMOD_PATTERN;
			auxCholmodSparse.itype = CHOLMOD_INT;
			auxCholmodSparse.dtype = CHOLMOD_DOUBLE;
			auxCholmodSparse.sorted = 1;
			auxCholmodSparse.packed = 1;
            // std::cout << "test" <<std::endl;
			int amdStatus = cholmod_amd(&auxCholmodSparse, NULL, 0, blockPermutation.data(), &m_cS);
			if (! amdStatus) {
				return false;
			}
            // std::cout << scalarPermutation.size() << " " << m_cholSparseS->ncol << std::endl;
			// blow up the permutation to the scalar matrix
			if (scalarPermutation.size() == 0)
				scalarPermutation.resize(m_cholSparseS->ncol);
			size_t scalarIdx = 0;
			int a = 0;
            
			for ( i = 0; i < m_ncams; ++i)
			{
				const int &pp = blockPermutation(i);
				int base = pp*3;
                // std::cout << pp << std::endl;
				// int nCols= (pp==0) ? 6 : 6;

                // int base =  pp*6-1;

				int nCols= 3;

				for ( j = 0; j < nCols; ++j)
					scalarPermutation(scalarIdx++) = base++;

			}
            // std::cout << scalarIdx << " " << m_cholSparseS->ncol << std::endl;

			assert(scalarIdx == m_cholSparseS->ncol);

			// apply the ordering
			m_cS.nmethods = 1 ;
			m_cS.method[0].ordering = CHOLMOD_GIVEN;
            // std::cout << scalarPermutation << std::endl;

			m_cholFactorS = cholmod_analyze_p(m_cholSparseS, scalarPermutation.data(), NULL, 0, &m_cS);
		}
        
		init = true;
	}
// std::cout << "test" << std::endl;
		
	//Cholmod package for solving sparse linear equation              
	cholmod_factorize(m_cholSparseS, m_cholFactorS, &m_cS); 
	m_cholSparseR = cholmod_solve (CHOLMOD_A, m_cholFactorS, m_cholSparseE, &m_cS) ;

// std::cout << "test" << std::endl;
// exit(1);

	return true;
}

int main()
{
    Mat pre_frame, cur_frame, pre_frame_gray, cur_frame_gray;
    
    VideoCapture cap("../data/Hamlyn/output.mp4");
    if (!cap.isOpened())
    {
        cout << "Error opening video stream or file" << endl;
    }
    Eigen::Matrix3d K;
    K <<    391.656525, 0.000000, 165.964371,
            0.000000, 426.835144, 154.498138,
            0.000000, 0.000000, 1.000000;
    double fx = K(0,0);
    double fy = K(1,1);
    double cx = K(0,2);
    double cy = K(1,2);
    std::vector<double> xyz;
    std::vector<int> faces;
    std::vector<int> new_faces;
    
    int number_vertices, number_faces;

    // Read Object file
    readObj(xyz, faces, number_vertices, number_faces);
    
    // for(int i=0;i<number_vertices;i++)
        
    
    
    int frameWidth = 360;
    int frameHeight = 288;
    int frameRate = 30;


    // Erstellen Sie einen VideoWriter, um das Video zu schreiben
    cv::VideoWriter video("output.avi", cv::VideoWriter::fourcc('M','J','P','G'), frameRate, cv::Size(frameWidth,frameHeight));


    cap >> pre_frame;

    // creating a mask
    
    cv::Mat hsvImage;
    cv::cvtColor(pre_frame, hsvImage, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsvImage, channels);
    cv::Mat preprocess_mask;
    int thresholdValue = 30; // Passen Sie den Schwellenwert nach Bedarf an
    cv::threshold(channels[2], preprocess_mask, thresholdValue, 255, cv::THRESH_BINARY);
    cv::imshow("Helligkeitsschwellenwert", preprocess_mask);
    // cv::waitKey(0);

    // preprocessing
    std::vector<cv::Point2f> p0;// = {point0, cv::Point2f(180,140), cv::Point2f(200, 200)};
    bool usable_point[number_vertices];

    verifyPoints(xyz, usable_point, number_vertices, K, preprocess_mask);
    
    int num_points = number_vertices;

    // fixing faces and points
    // char* v_mask = (char*)malloc(num_points*num_points*sizeof(char)); // v_mask initialisieren!
    // memset( v_mask, 0, num_points*num_points*sizeof(char) );
    bool usabale_faces[number_faces];
    int num_faces = 0;
    double *vertices;
    // remapping(faces, vertices, usabale_faces, number_faces);
    remapping(xyz, faces, vertices, usabale_faces, usable_point, number_faces, num_points);
    double *reference;
    for(int i=0;i<num_points;i++)
        reference[i] = vertices[i];

    num_faces = number_faces;
    // std::cout << num_points << std::endl;
    // exit(1);
    char* v_mask = (char*)malloc(num_points*num_points*sizeof(char)); // v_mask initialisieren!
    memset( v_mask, 0, num_points*num_points*sizeof(char) );

    for(int i=0; i < num_faces;i++) {
        int f1 = new_faces[i*3];
        int f2 = new_faces[i*3+1];
        int f3 = new_faces[i*3+2];

        v_mask[f1*num_points+f1] = 1;
        v_mask[f2*num_points+f2] = 1;
        v_mask[f3*num_points+f3] = 1;

        if(f1 < f2)
            v_mask[f1*num_points+f2] = 1;
        else
            v_mask[f2*num_points+f1] = 1;

        if(f1 < f3)
            v_mask[f1*num_points+f3] = 1;
        else
            v_mask[f3*num_points+f1] = 1;

        if(f2 < f3)
            v_mask[f2*num_points+f3] = 1;
        else
            v_mask[f3*num_points+f2] = 1;
        
            
    }

    // std::cout << counter << std::endl;

    // creating observable points
    double alp[] = {0,0,1,0.3333,0.1,0.1,0.8};
    double bet[] = {0,1,0,0.3333,0.1,0.8,0.1};
    double gam[] = {1,0,0,0.3333,0.8,0.1,0.1};



    std::vector<double> obs;
    int num_obs = 0;
    
    for(int face_id=0;face_id < num_faces; face_id++) {
        
        int f1 = new_faces[face_id*3];
        int f2 = new_faces[face_id*3+1];
        int f3 = new_faces[face_id*3+2];
        
        for(int id=0; id<7;id++) {
            double v1[] = {vertices[f1*3 + 0], vertices[f1*3 + 1], vertices[f1*3 + 2]};
            double v2[] = {vertices[f2*3 + 0], vertices[f2*3 + 1], vertices[f2*3 + 2]};
            double v3[] = {vertices[f3*3 + 0], vertices[f3*3 + 1], vertices[f3*3 + 2]};

            double alpha = alp[id];
            double beta = bet[id];
            double gamma = gam[id];
            if ((alpha +beta+gamma >1) || (alpha +beta+gamma < 0))
                return 1;
            double p[3] = { v1[0] * alpha + v2[0]*beta + v3[0]*gamma,
                            v1[1] * alpha + v2[1]*beta + v3[1]*gamma,
                            v1[2] * alpha + v2[2]*beta + v3[2]*gamma};

            double x = p[0];
            double y = p[1];
            double z = p[2];

            double u = fx*x/z + cx;
            double v = fy*y/z + cy;

            obs.push_back(face_id);
            obs.push_back(u);
            obs.push_back(v);
            obs.push_back(alpha);
            obs.push_back(beta);
            obs.push_back(gamma);
            p0.push_back(cv::Point2f(int(u),(v)));
            cout << (u) << " " << (v) << " " << int(f1) << " " << int(f2) << " " << int(f3) << endl;
            num_obs++;
        }
    }
    

    // init optimizer
    // char* v_mask = (char*)malloc(num_points*sizeof(char)); // v_mask initialisieren!
    // memset( v_mask, 0, num_points*sizeof(char) );

    struct sba_crsm Sidxij;
    int nnz=0;
    // std::cout << num_points << std::endl;
    constructSmask( Sidxij, num_points, nnz, v_mask); //, Uidxij, v_mask);

    double* V	=	(double *)malloc(nnz * 3 * 3 * sizeof(double));
    double* g	=	(double *)malloc(num_points * 3 * sizeof(double));
    double* dx	=	(double *)malloc((num_points*3)*sizeof(double));
    double* error = (double *)malloc(((num_obs*2) + (num_faces*3))*sizeof(double));

    // memset( V, 0, nnz * 3 * 3 * sizeof(double));
    // memset( g, 0, num_points * 3 * sizeof(double));
    // memset( dx, 0, (num_points*3)*sizeof(double));

// compute_reprojection_error(error, obs.data(), xyz.data(), faces.data(), K, num_obs);
    // main loop
    cvtColor(pre_frame, pre_frame_gray, COLOR_BGR2GRAY);

    Mat mask = Mat::zeros(pre_frame_gray.size(), pre_frame_gray.type());
    vector<Scalar> colors;
    colors.push_back(Scalar(255,0,0));
    // Read the frames to the last frame

    cholmod_start (&m_cS) ; 
    int *Ap  = (int*)malloc((num_points + 1)*sizeof(int));
	int * Aii = (int*)malloc(nnz*sizeof(int));;
	constructAuxCSSGN( Ap, Aii, v_mask, num_points );

     m_cholSparseE = cholmod_zeros( 3*num_points, 1, CHOLMOD_REAL, &m_cS); // Achtung! Warum sieben?
	double* Ex = (double*)m_cholSparseE->x;
	int nMaxS = (nnz-num_points)*9+num_points*6;	//maximum non-zero element in S matrix 

    m_cholSparseS = cholmod_allocate_sparse(num_points*3,num_points*3,nMaxS,true,true,1,CHOLMOD_REAL,&m_cS); 
    
	int *Sp, *Si;
	double* Sx = NULL;
	Sp = (int*)m_cholSparseS->p;		//column pointer
	Si = (int*)m_cholSparseS->i;		//row pointer
    bool init = false;
    bool ordering = true;
    double *rx;

    // for(int i=0;i<num_faces;i++) {
    //     std::cout << new_faces[i] << std::endl;
    // }exit(1);
    // double pts[num_points*3];
    for(int i=0; i< num_points*3; i++) {
            vertices[i] += 0.00*i;
        }

    while (1)
    {
        // Initialise frame matrix
        
        cap >> cur_frame;
        
        if(cur_frame.empty())
            break;

        // feature tracking!
        cvtColor(cur_frame, cur_frame_gray, COLOR_BGR2GRAY);
        std::vector<cv::Point2f> p1;
        vector<uchar> status;
        vector<float> err;
        TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
        calcOpticalFlowPyrLK(pre_frame_gray, cur_frame_gray, p0, p1, status, err, Size(360/4,288/4),10, criteria); // also 21,21 window would be good
        vector<Point2f> good_new;

        // std::cout << status.size() << std::endl;
        for(int i=0; i< num_obs; i++) {
            obs[i*6+1] = double(p1[i].x); // was ist die zahlen einheit?? ist die zahl richtig?
            obs[i*6+2] = double(p1[i].y);
            
        }
        // for(int i=0; i< num_points*3; i++) {
        //     vertices[i] += 0.1;
        // }

        for(uint i = 0; i < p0.size(); i++)
        {
            // Select good points
            if(1) { // status[i] == 1
                // good_new.push_back(p1[i]); 
                // Draw the tracks
                line(cur_frame, p1[i], p0[i], Scalar(0, 255, 0), 1);
                circle(cur_frame, p1[i], 1, Scalar(0, 0, 255), -1);
            }
        }

        //optimization
        for (int iter=1;iter<10;iter++) {
            memset( error, 0, (((num_obs*2) + (num_faces*3))*sizeof(double) ));
            memset( V, 0, nnz * 3 * 3 * sizeof(double));
            memset( g, 0, num_points * 3 * sizeof(double));
            memset( dx, 0, (num_points*3)*sizeof(double));

            compute_reprojection_error(error, obs.data(), vertices, new_faces.data(), K, num_obs);
            compute_distance_error(error, vertices, reference, new_faces.data(), num_faces, num_obs);

            compute_reprojection_jacobian(V, g, error, obs.data(), vertices, new_faces.data(), K, num_obs, Sidxij);
            compute_distance_jacobian(V,g,error, vertices, new_faces.data(), num_faces, num_obs, Sidxij);


        
            constructCSSGN( Si, Sp, Sx, V, Sidxij, init, v_mask, num_points); //set CSS format using S matrix
            for(int ii=0; ii< num_points*3; ii++) {
                Ex[ii] = g[ii];
            //    cout << g[ii] << endl;
            }
            // exit(1);
            solveCholmodGN( Ap, Aii, init, ordering, num_points, nnz);

            

            init = true;
            rx = (double*)m_cholSparseR->x;
            if (m_cS.status == CHOLMOD_NOT_POSDEF)
            {
                printf ( "Cholesky failure, writing debug.txt (Hessian loadable by Octave)" );
                exit(1);
            }
            double dx = 0;
            for(int i=0; i < num_points; i++) {
                vertices[i] += rx[i];
                dx += rx[i]*rx[i];
            }
            double cost = 0;
            for(int i=0; i < ((num_obs*2) + (num_faces*3)); i++) {
                cost += error[i] * error[i];
            }
            double er=0;
            for(int i=0; i < ((num_obs*2)); i++) {
                er += error[i] * error[i];
            }
            er /= num_obs*2;
            double ed = 0;
            for(int i=(num_obs*2); i < ((num_obs*2) + (num_faces*3)); i++) {
                ed += error[i] * error[i];
            }
            ed /= num_faces*3;
            
            cost /= ((num_obs*2) + (num_faces*3));
            std::cout << "Itertation: " << iter <<" Error: " << cost << " dx: " << dx / num_points <<" er: " << er << " ed: " << ed << std::endl;

            if((cost < 0.0001) || (dx < 0.0001))
                break;
        }
        
        imshow("Frame", cur_frame);
        // video.write(cur_frame);
        //wait 20 ms between successive frames and break the loop if key q is pressed
        int key = waitKey(1000);
        if (key == 'q')
        {
            cout << "q key is pressed by the user. Stopping the video" << endl;
            break;
        }

        // optimisation
 
 
  }


	sba_crsm_free(&Sidxij);

	cholmod_free_factor(&m_cholFactorS, &m_cS) ;              
	cholmod_l_free_dense(&m_cholSparseE, &m_cS);
	cholmod_l_free_dense(&m_cholSparseR, &m_cS);
	cholmod_finish (&m_cS) ;  
	free(Ap);
	free(Aii);
	cholmod_free_sparse(&m_cholSparseS, &m_cS) ;
    free(V);
    free(g);
    free(error);
    free(dx);
  // Release the video capture object
  cap.release();
  destroyAllWindows();
  return 0;
}