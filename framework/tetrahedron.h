//------------------------------------------------
// TETRAHEDRON DECLARATIONS
//------------------------------------------------

#ifndef TETRAHEDRON_H_
#define TETRAHEDRON_H_

#include "point.h"
#include "triangle.h"
#include "vector3.h"
#include "vector4.h"

namespace BOB{
/*
 
	Cell:
 			2 -------- 3
 		   /|		  /|
 		  /	|	     / |
 		 6 -------- 7  |
         |  |	    |  |
 		 |  0 ------|- 1
         | /	    | /
         |/		    |/
 		 4 -------- 5
 
 	Tets:
 		case 0:
                      |				|			  |					|
 			2         |   2 --- 3	|     2		  |					|	  2 --- .
 			|		  |	   	   /|	|    /		  |					|	 /|	   /|
 			|		  |		  7 |	|   6 --- 7	  |			7		|	. --- 7 |
 			0 --- 1   |	    	1	|   |		  |			| 1		|	| .	  | 1
           /		  |				|   |		  |			|/		|	|	  |/
          4			  |				|   4		  |   4 --- 5   	|	4 --- .
 
 		case 1:
                      |				|			  |					|
 			      3   |   2 --- 3	|     		3 |					|	  . --- 3
                  |	  |	 /|			|    	   /  |					|	 /|	   /|
 			      |	  | 6 |			|   6 --- 7	  |	    6			|	6 --- . |
 			0 --- 1   |	  0			|   	  |	  |	    | 0		    |	| 0	  | .
           		 /	  |				|  		  |	  |	    |/		    |	|	  |/
          		5	  |				|  		  4   |     4 --- 5     |	. --- 5

 
 */

enum Tet_rotation {
    abcd,acdb,adbc,badc,bcad,bdca,
    cabd,cbda,cdab,dacb,dbac,dcba
};


const int NADJ[12]={0,2,1,1,0,3,0,3,2,2,1,3};

const Tet_rotation TADJ[12]={
    abcd,bcad,cabd,abcd,cabd,bcad,
    bcad,cabd,abcd,cabd,bcad,abcd
};

const Tet_rotation RADJ[12]={
    abcd,cabd,bcad,abcd,bcad,cabd,
    cabd,bcad,abcd,bcad,cabd,abcd
};

const Tet_rotation MULT[12][12]={
    abcd,acdb,adbc,badc,bcad,bdca,cabd,cbda,cdab,dacb,dbac,dcba,
    acdb,adbc,abcd,bdca,badc,bcad,cbda,cdab,cabd,dcba,dacb,dbac,
    adbc,abcd,acdb,bcad,bdca,badc,cdab,cabd,cbda,dbac,dcba,dacb,
    badc,cabd,dacb,abcd,cbda,dbac,acdb,bcad,dcba,adbc,bdca,cdab,
    bcad,cdab,dbac,adbc,cabd,dcba,abcd,bdca,dacb,acdb,badc,cbda,
    bdca,cbda,dcba,acdb,cdab,dacb,adbc,badc,dbac,abcd,bcad,cabd,
    cabd,dacb,badc,dbac,abcd,cbda,bcad,dcba,acdb,cdab,adbc,bdca,
    cbda,dcba,bdca,dacb,acdb,cdab,badc,dbac,adbc,cabd,abcd,bcad,
    cdab,dbac,bcad,dcba,adbc,cabd,bdca,dacb,abcd,cbda,acdb,badc,
    dacb,badc,cabd,cbda,dbac,abcd,dcba,acdb,bcad,bdca,cdab,adbc,
    dbac,bcad,cdab,cabd,dcba,adbc,dacb,abcd,bdca,badc,cbda,acdb,
    dcba,bdca,cbda,cdab,dacb,acdb,dbac,adbc,badc,bcad,cabd,abcd
};

const Tet_rotation REVR[12]={
    abcd,adbc,acdb,badc,cabd,dacb,
    bcad,dbac,cdab,bdca,cbda,dcba
};

const int Tet_vertices[12][4]={
    0,1,2,3, 0,2,3,1, 0,3,1,2, 1,0,3,2,
    1,2,0,3, 1,3,2,0, 2,0,1,3, 2,1,3,0,
    2,3,0,1, 3,0,2,1, 3,1,0,2, 3,2,1,0
};

inline Tet_rotation operator*(Tet_rotation a,Tet_rotation b){ return MULT[b][a];}

inline float ScTP(const Vector3 &a, const Vector3 &b, const Vector3 &c)
{
    // computes scalar triple product
    //return Dot(a, Cross(b, c));
    return a.dot(b.cross(c));
}

inline Vector4 Tet_barycentric(const Vector3 & a, const Vector3 & b, const Vector3 & c, const Vector3 & d, const Vector3 & p)
{
    Vector3 vap = p - a;
    Vector3 vbp = p - b;
    
    Vector3 vab = b - a;
    Vector3 vac = c - a;
    Vector3 vad = d - a;
    
    Vector3 vbc = c - b;
    Vector3 vbd = d - b;
    
    // ScTP computes the scalar triple product
    float va6 = ScTP(vbp, vbd, vbc);
    float vb6 = ScTP(vap, vac, vad);
    float vc6 = ScTP(vap, vad, vab);
    float vd6 = ScTP(vap, vab, vac);
    float v6 = 1 / ScTP(vab, vac, vad);
    return Vector4(va6*v6, vb6*v6, vc6*v6, vd6*v6);
}

class Tetrahedron{
    class Tet_adj_ptr;
    class Tet_ptr
    {	public:
        Tetrahedron *P;
        Tet_rotation R;
        Tet_ptr(Tetrahedron* p=0,Tet_rotation r=abcd):P(p),R(r){}
        Tet_ptr(Tet_adj_ptr& A):P(A.tet.P->adjacents[NADJ[A.tet.R]].P),R(A.tet.P->adjacents[NADJ[A.tet.R]].R*TADJ[A.tet.R]){}
        Tet_ptr operator*(Tet_rotation r){ return Tet_ptr(P,R*r);}
        Point*& operator[](int n){ return P->vertices[Tet_vertices[R][n]];}
        Tet_adj_ptr Adj(){ return Tet_adj_ptr(*this);}
        Tet_ptr Abcd(){ return *this;}
        Tet_ptr Acdb(){ return *this*acdb;}
        Tet_ptr Adbc(){ return *this*adbc;}
        Tet_ptr Badc(){ return *this*badc;}
        Tet_ptr Bcad(){ return *this*bcad;}
        Tet_ptr Bdca(){ return *this*bdca;}
        Tet_ptr Cabd(){ return *this*cabd;}
        Tet_ptr Cbda(){ return *this*cbda;}
        Tet_ptr Cdab(){ return *this*cdab;}
        Tet_ptr Dacb(){ return *this*dacb;}
        Tet_ptr Dbac(){ return *this*dbac;}
        Tet_ptr Dcba(){ return *this*dcba;}
    };
    class Tet_adj_ptr
    {	public:
        Tet_ptr &tet;
        Tet_adj_ptr(Tet_ptr&T):tet(T){}
        void operator=(Tet_ptr& T){ tet.P->adjacents[NADJ[tet.R]].P=T.P; tet.P->adjacents[NADJ[tet.R]].R=T.R*RADJ[tet.R];}
        Tet_ptr Adj(){ return tet;}
        Point*& operator[](int n){ return Tet_ptr(*this)[n];}
        Tet_ptr Abcd(){ return Tet_ptr(*this).Abcd();}
        Tet_ptr Acdb(){ return Tet_ptr(*this).Acdb();}
        Tet_ptr Adbc(){ return Tet_ptr(*this).Adbc();}
        Tet_ptr Badc(){ return Tet_ptr(*this).Badc();}
        Tet_ptr Bcad(){ return Tet_ptr(*this).Bcad();}
        Tet_ptr Bdca(){ return Tet_ptr(*this).Bdca();}
        Tet_ptr Cabd(){ return Tet_ptr(*this).Cabd();}
        Tet_ptr Cbda(){ return Tet_ptr(*this).Cbda();}
        Tet_ptr Cdab(){ return Tet_ptr(*this).Cdab();}
        Tet_ptr Dacb(){ return Tet_ptr(*this).Dacb();}
        Tet_ptr Dbac(){ return Tet_ptr(*this).Dbac();}
        Tet_ptr Dcba(){ return Tet_ptr(*this).Dcba();}
    };
   public: 
    Point* vertices[4];
    Tet_ptr adjacents[4];
    

    Point* operator[](int n){ return vertices[n];}
    Tetrahedron(Point* v0=0,Point* v1=0,Point* v2=0,Point* v3=0);
    
};

inline Tetrahedron::Tetrahedron(Point* v0,Point* v1,Point* v2,Point* v3)
{
    vertices[0]=v0; vertices[1]=v1; vertices[2]=v2; vertices[3]=v3;
    adjacents[0]=0; adjacents[1]=0; adjacents[2]=0; adjacents[3]=0;
}
} // end namespace BOB
#endif /* TETRAHEDRON_H_ */
