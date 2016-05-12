#include "Ridesharing/privacy.h"

#include <chrono>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <boost/format.hpp>
#include "bloom_filter.h"
#include "/home/vagrant/nfllib/include/nfl.hpp"
#include "/home/vagrant/nfllib/include/nfl/params.hpp"


//#define memset_s(W,WL,V,OL) memset(W,V,OL)
//#include "/Users/mkilliji/Code/keccak-tiny/keccak-tiny.c"

#define CONFIG 512, 62, uint64_t

template <class P>
__attribute__((noinline)) static void encrypt(P& a, P& b, P const & message, uint64_t pt_modulus, P const & pka, P const & pkb, P const & pkaprime, P const & pkbprime, FastGaussianNoise<uint8_t, typename P::value_type, 2> *g_prng)
{
    // u
    P e1 = nfl::gaussian<typename P::value_type>(g_prng);
    e1.ntt_pow_phi();
  
    // p*e_1
    P e2 = nfl::gaussian<typename P::value_type>(g_prng, pt_modulus);
    e2.ntt_pow_phi();

    // p*e_2
    P e3 = nfl::gaussian<typename P::value_type>(g_prng, pt_modulus);
    e3 = e3 + message;
    e3.ntt_pow_phi();

    // encCset_a = pka * u + p*e_2
    a = e1 * pka + e2 ;

    // encCset_b = pkb * u + p*e_3 + message
    b = e1 * pkb + e3;
}

template <class P>
__attribute__((noinline)) static void decrypt(P& tmp, P const & a, P const& b, P const& s, P const& sprime, typename P::value_type const modulus, uint64_t const pt_modulus)
{
    using fuck = typename P::signed_value_type;
    tmp = b -  nfl::shoup(a * s, sprime);
  
    tmp.invntt_pow_invphi();
    
    uint64_t const adder=(uint64_t)(((1ULL<<63)/pt_modulus)*pt_modulus-modulus);
  
    for(auto & v : tmp)
    {
        v= (v<modulus/2) ? v%pt_modulus : (v + adder)%pt_modulus;
    }
  
}

template <class P, class Q>
static void plonge(P& out, Q& in) {
    for(int j=0;j<out.nmoduli;j++) {
        for(int i=0;i<out.degree;i++) {
            out(j,i)=in(j,i);
        }
    }
}

template <class P>
static void encode(P& res, unsigned char const bits[],  int const size) { 
    using poly_cleartext = nfl::poly_from_modulus<uint16_t, 512, 14>;
    poly_cleartext tmp=0;
    for(int j=0;j<tmp.nmoduli;j++) {
        for(int i=0;i<tmp.degree&&i<size;i++) {
            tmp(j,i)=bits[i];
        }
    }
    tmp.invntt_pow_invphi();
    plonge(res,tmp);
}
 
template <class P>
static void decode(unsigned char *& clear, P const res, int const size) {
    using poly_cleartext = nfl::poly_from_modulus<uint16_t, 512, 14>;
    poly_cleartext tmp=0;
    plonge(tmp,res);
    tmp.ntt_pow_phi();
    clear=(unsigned char *)malloc(size*sizeof(int));
    for(int i=0;i<res.degree&&i<size;i++) {
        clear[i]=tmp(0,i);
    }
}

template <class P>
static void decode_noalloc( char  * clear, P const res, int const size) {
    using poly_cleartext = nfl::poly_from_modulus<uint16_t, 512, 14>;
    poly_cleartext tmp=0;
    plonge(tmp,res);
    tmp.ntt_pow_phi();
    for(int i=0;i<res.degree&&i<size;i++) {
        clear[i]=tmp(0,i);
    }
}

template <class P>
static void encode_nontt(P& res, uint8_t* const bits,  int const size) { 
    using poly_cleartext = nfl::poly_from_modulus<uint16_t, 512, 14>;
    poly_cleartext tmp=0;
    for(int j=0;j<tmp.nmoduli;j++) {
        for(int i=0;i<tmp.degree&&i<size;i++) {
            tmp(j,i)=bits[i];
        }
    }
    plonge(res,tmp);
}
 
uint64_t cleartext_modulus() {
    using poly_cleartext = nfl::poly_from_modulus<uint16_t, 512, 14>;
    return poly_cleartext::get_modulus(0);
}


template <class T>
double get_time_us(T const& start, T const& end, uint32_t N)
{
    auto diff = end-start;
    return (long double)(std::chrono::duration_cast<std::chrono::microseconds>(diff).count())/N;
}

template <class T, size_t Align, class... Args>
T* alloc_aligned(size_t n, Args&& ... args)
{
    T* ret;
    if (posix_memalign((void**) &ret, Align, sizeof(T)*n) != 0) {
        throw std::bad_alloc();
    }
    for (size_t i = 0; i < n; i++) {
        new (&ret[i]) T{std::forward<Args>(args)...};
    }
    return ret;
}

template<size_t degree, size_t modulus, class T>
int testencode_decode () {
    using poly_cyphertext = nfl::poly_from_modulus<T, degree, modulus>;
    
    uint8_t in[degree];
    fastrandombytes(in,degree);
    for(int i=0;i<degree;i++) {
        in[i]=(in[i])%2;
    }
    
    uint8_t  *out;
    poly_cyphertext c;
    encode(c,in,degree);
    decode(out, c,degree);
    for(int i=0;i<degree;i++) {
        assert(in[i] == out[i]);
    }
    
    std::cout << "======================================================================" << std::endl;
    std::cout << "Encode/Decode test passed" << std::endl;
    
}

template<size_t degree, size_t modulus, class T>
int testencrypt_decrypt () {
    using poly_cyphertext = nfl::poly_from_modulus<T, degree, modulus>;
    poly_cyphertext &pka = *alloc_aligned<poly_cyphertext, 32>(1),
    &pkaprime = *alloc_aligned<poly_cyphertext, 32>(1),
    &pkb = *alloc_aligned<poly_cyphertext, 32>(1),
    &pkbprime = *alloc_aligned<poly_cyphertext, 32>(1),
    &a = *alloc_aligned<poly_cyphertext, 32>(1),
    &b = *alloc_aligned<poly_cyphertext, 32>(1),
    &d = *alloc_aligned<poly_cyphertext, 32>(1);
                    
    // This step generates a secret key
    FastGaussianNoise<uint8_t, T, 2> g_prng(8, 128, 1<<10);

    poly_cyphertext &s = *alloc_aligned<poly_cyphertext, 32>(1, nfl::gaussian<T>(&g_prng));
    poly_cyphertext &sprime = *alloc_aligned<poly_cyphertext, 32>(1);
    s.ntt_pow_phi();
    sprime = nfl::compute_shoup(s);

    // This step generates a public key
    pka = nfl::uniform();
    pkb = nfl::gaussian<T>(&g_prng, cleartext_modulus());
    pkb.ntt_pow_phi();

    // pkb = pkb + pka * s;
    pkb = pkb + nfl::shoup(pka * s, sprime);
    pkaprime = nfl::compute_shoup(pka);
    pkbprime = nfl::compute_shoup(pkb);
    
    uint8_t in[degree];
    fastrandombytes(in,degree);
    for(int i=0;i<degree;i++) {
        in[i]=(in[i])%2;
    }
    
    uint8_t  *out;
    poly_cyphertext c;
    encode(c,in,degree);
    
    encrypt(a, b, c, cleartext_modulus(), pka, pkb, pkaprime, pkbprime, &g_prng);
    decrypt(d, a, b, s, sprime, d.get_modulus(0),cleartext_modulus());
    
    
    decode(out, d,degree);
    for(int i=0;i<degree;i++) {
        assert(in[i] == out[i]);
    }
    
    std::cout << "======================================================================" << std::endl;
    std::cout << "Encrypt/Decrypt test passed" << std::endl;
    
}

// The PSI protocol is simple
// Remember it is an asymetric protocol where C learns the intersection of its set with S's set
// The protocol isn't secure against malicious participants, i.e. if C's set = Universe then Intersection(Universe,S's set)=S's set
// --> C's can learn S's set
// The protocol is secure with honest but curious (semi-honest) participants
// C calls initiateC with its set
// sends the resulting char buffer to S (which includes C's public key)
// S calls initiateS with its set and the received buffer
// sends back the resulting char buffer to C
// C calls terminateC with the received buffer
// and computes the intersection
// As classical with PSI, it is an asymetric protocol, i.e. C learns the intersection but not S
// if you want it to be symetrical, just do it also the other way around, i.e. C becomes S and vice versa (see testSymetricPSIAsLib as an example)

// initiateC is called by C to launch the protocol
template<size_t degree, size_t modulus, class T>
void initiateC(char*& outbuf,char* const inbuf,uint64_t  &outlen, uint64_t const inlen, nfl::poly_from_modulus<T, degree, modulus> *&SecretKey){
    
    // nbP is the number of polynoms we are going to create
    uint64_t nbP=(inlen/degree)+ ( (inlen%degree==0) ? 0 : 1 );
    
    // If size doesn't divide degree, we have to increase the inbuf size
    // and then recursively call initiateC (next time the buffer size is going to divide degree and then no recursion)
    if((inlen%degree!=0)) {
        char *new_inbuf=(char*)malloc(nbP*degree);
        memcpy(new_inbuf,inbuf,inlen);
        initiateC<degree,modulus,T>(outbuf,new_inbuf,outlen,nbP*degree,SecretKey);
    } else {
        
    using poly_cyphertext = nfl::poly_from_modulus<T, degree, modulus>;
    
    // the input is in inbuf, has length inlen

    // Allocate a large enough buffer for :
    // the public keys (and their shoupified versions) and the two (times nbP) ciphers a and b
    outlen = (nbP*2+4)*degree*sizeof(T);
    outbuf=(char*)malloc(outlen);
    
    poly_cyphertext *pka = (poly_cyphertext*)outbuf,
                    *pkaprime = (poly_cyphertext*)(outbuf+degree*sizeof(T)),
                    *pkb = (poly_cyphertext*)(outbuf+2*degree*sizeof(T)),
                    *pkbprime = (poly_cyphertext*)(outbuf+3*degree*sizeof(T));
                    
    // This step generates a secret key
    FastGaussianNoise<uint8_t, T, 2> g_prng(8, 128, 1<<10);
    SecretKey = alloc_aligned<poly_cyphertext, 32>(1, nfl::gaussian<T>(&g_prng));
    SecretKey->ntt_pow_phi();

    // This step generates a public key
    // pka is just uniform
    *pka = nfl::uniform();
    *pkb = nfl::gaussian<T>(&g_prng, cleartext_modulus());
    pkb->ntt_pow_phi();
    // pkb = pkb + pka * s;
    *pkb = *pkb + *pka * *SecretKey;
    *pkaprime = nfl::compute_shoup(*pka);
    *pkbprime = nfl::compute_shoup(*pkb);
    
    // Then alloc a single polynom for encoding and encrypting in the loop
    poly_cyphertext p_in=*alloc_aligned<poly_cyphertext, 32>(1);
    
    // Then do the encoding and encrypting in the loop
    for(int i=0;i<nbP;i++) {
        encode(p_in,(unsigned char*)(inbuf+i*degree),degree);
        // encrypt (a[i]),b[i],p_in,...) but we are doing this directly in the output buffer
        encrypt(*((poly_cyphertext*)(outbuf+(4+i)*degree*sizeof(T))), *((poly_cyphertext*)(outbuf+(nbP+4+i)*degree*sizeof(T))), p_in, cleartext_modulus(), *pka, *pkb, *pkaprime, *pkbprime, &g_prng);
    }}
}

// initiateS is called by S upon receiving a request by C, that's the second step of the protocol
template<size_t degree, size_t modulus, class T>
void initiateS(char*& outbuf,char* const inbuf, char* Sinput, uint64_t &outlen, uint64_t const inlen){
    // nbP is the number of polynoms we are going to create
    uint64_t nbP=(inlen/degree)+ ( (inlen%degree==0) ? 0 : 1 );
    
    using poly_cyphertext = nfl::poly_from_modulus<T, degree, modulus>;
    
    // C sent its public keys in the input buffer (pka,pkb,pkaprime,pkbprime)
    poly_cyphertext *pka = (poly_cyphertext*)inbuf,
                    *pkaprime = (poly_cyphertext*)(inbuf+degree*sizeof(T)),
                    *pkb = (poly_cyphertext*)(inbuf+2*degree*sizeof(T)),
                    *pkbprime = (poly_cyphertext*)(inbuf+3*degree*sizeof(T)),
                    // and we are going to produce a cipher of a zero polynom
                    &p_zero = *alloc_aligned<poly_cyphertext, 32>(1),
                    &zeroa = *alloc_aligned<poly_cyphertext, 32>(1),
                    &zerob = *alloc_aligned<poly_cyphertext, 32>(1);
                        
    FastGaussianNoise<uint8_t, T, 2> g_prng(8, 128, 1<<10);
                    
    // We are going to encode S input set into p_inS
    poly_cyphertext p_inS=*alloc_aligned<poly_cyphertext, 32>(1);
    
    // The output buffer and its length
    outlen = (nbP*2)*degree*sizeof(T);
    outbuf=(char*)malloc(outlen);
    
    // A zero polynom that is going to be ciphered in the loop.
    // Please note that it could be sent by C and S would not need C's public key
    // But it is more secure to produce a new zero polynom for each addition
    // Henceforth by enabling S to produce them (by giving it S public key)
    // We save on communication, but not on computation
    char zero[degree];
    for(int i=0;i<degree;i++) {
        zero[i]=0;
    }
    encode(p_zero,(unsigned char*)zero,degree);
    
    for(int i=0;i<nbP;i++) {
        // S's set is encoded into p_inS and NTTified before mult
        encode(p_inS,(unsigned char*)(Sinput+i*degree),degree);
        p_inS.ntt_pow_phi();
        // We produce a new cipher of zero
        encrypt(zeroa, zerob, p_zero, cleartext_modulus(),*pka, *pkb, *pkaprime, *pkbprime, &g_prng);
        // and compute the intersection and add zero
        // a = a *p_ins + zero and b =b * p_ins +zero
        *((poly_cyphertext*)(outbuf+i*degree*sizeof(T))) = *((poly_cyphertext*)(inbuf+(4+i)*degree*sizeof(T))) * p_inS + zeroa;  
        *((poly_cyphertext*)(outbuf+(i+nbP)*degree*sizeof(T))) = *((poly_cyphertext*)(inbuf+(4+i+nbP)*degree*sizeof(T))) * p_inS + zerob;  
    }
}

// terminateC is called by C upon receiving a reply by S, that's the third and final step of the protocol
template<size_t degree, size_t modulus, class T>
void terminateC(char* const outbuf,char* const inbuf,uint64_t outlen, uint64_t const inlen, nfl::poly_from_modulus<T, degree, modulus> *SecretKey){
    // nbP is the number of polynoms we are going to create
    uint64_t nbP=(inlen/degree)+ ( (inlen%degree==0) ? 0 : 1 );

    using poly_cyphertext = nfl::poly_from_modulus<T, degree, modulus>;
    
    // Sprime is the shoupified secret key
    poly_cyphertext &sprime = *alloc_aligned<poly_cyphertext, 32>(1);
    sprime = nfl::compute_shoup(*SecretKey);
    
    // d is a polynom into which we are going to decrypt in the loop
    poly_cyphertext &d=*alloc_aligned<poly_cyphertext, 32>(1);

    // TODO: Do we really need to compute this apart from stats ?
    outlen=nbP*degree*sizeof(T);

    for(int i=0;i<nbP;i++) {
        // We decrypt S's reply into d (decrypt(d,a,b,...))
        decrypt(d, *(poly_cyphertext*)(inbuf+i*degree*sizeof(T)), *(poly_cyphertext*)(inbuf+(nbP+i)*degree*sizeof(T)), *SecretKey, sprime, d.get_modulus(0),cleartext_modulus());
        // And decode d into outbuf
        //  the length of the decode depends on whether or not the size of the universe fits into degree's polynoms
        if(i==nbP-1&&inlen%degree!=0)
            decode_noalloc((outbuf+i*degree*sizeof(char)), d ,inlen%degree);
        else
            decode_noalloc((outbuf+i*degree*sizeof(char)), d ,degree);
    }
    
}

static char* getRandomBinaryBytes(uint64_t size) {
    unsigned char *buff=(unsigned char*)malloc(size);
    fastrandombytes(buff,size);
    for(int i=0;i<size;i++) {
        buff[i]=(buff[i])%2;
    }
    return (char*)buff;
}

template<size_t degree, size_t modulus, class T>
int testPSI1(uint64_t size)
{
    nfl::poly_from_modulus<T, degree, modulus> *SecretKey;
    
    char *Cset=getRandomBinaryBytes(size);
    uint8_t Sset[size];
    for(int i=0;i<size;i++) {
        Sset[i]=1;
    }
    
    char *Cout;
    uint64_t Coutlen;
    initiateC<degree,modulus,T>(Cout,(char*)Cset,Coutlen,size,SecretKey);

    char *Sout;
    uint64_t Soutlen;
    initiateS<degree,modulus,T>(Sout,Cout,(char*)Sset,Soutlen,size);
    
    char Intersection[size];
    terminateC<degree,modulus,T>(Intersection,Sout,Soutlen,size,SecretKey);
    
    for(int i=0;i<size;i++) {
        assert(Intersection[i]==Cset[i]);
    }

    std::cout << "======================================================================" << std::endl;
    std::cout << "PSI test1 passed" << std::endl;
    
}

template<size_t degree, size_t modulus, class T>
int testPSI0(uint64_t size)
{
    nfl::poly_from_modulus<T, degree, modulus> *SecretKey;
    
    char *Cset=getRandomBinaryBytes(size);

    uint8_t Sset[size];
    for(int i=0;i<size;i++) {
        Sset[i]=0;
    }
    
    char *Cout;
    uint64_t Coutlen;
    initiateC<degree,modulus,T>(Cout,(char*)Cset,Coutlen,size,SecretKey);

    char *Sout;
    uint64_t Soutlen;
    initiateS<degree,modulus,T>(Sout,Cout,(char*)Sset,Soutlen,size);
    
    char Intersection[size];
    terminateC<degree,modulus,T>(Intersection,Sout,Soutlen,size,SecretKey);
    
    for(int i=0;i<size;i++) {
        assert(Intersection[i]==0);
    }
    
    std::cout << "======================================================================" << std::endl;
    std::cout << "PSI test0 passed" << std::endl;
    
    
}

template<size_t degree, size_t modulus, class T>
int testPSIAND(uint64_t size)
{
    nfl::poly_from_modulus<T, degree, modulus> *SecretKey;
    
    char *Cset=getRandomBinaryBytes(size);
    char *Sset=getRandomBinaryBytes(size);
    
    char *Cout;
    uint64_t Coutlen;
    initiateC<degree,modulus,T>(Cout,(char*)Cset,Coutlen,size,SecretKey);
    
    char *Sout;
    uint64_t Soutlen;
    initiateS<degree,modulus,T>(Sout,Cout,(char*)Sset,Soutlen,size);
    
    char Intersection[size];
    terminateC<degree,modulus,T>(Intersection,Sout,Soutlen,size,SecretKey);
    
    for(int i=0;i<size;i++) {
        assert(Intersection[i]==(Cset[i]&&Sset[i]));
    }
    
    std::cout << "======================================================================" << std::endl;
    std::cout << "PSI testAND passed" << std::endl;
    
    
}

template<size_t degree, size_t modulus, class T>
int testPSIAsLib(uint64_t size)
{
    
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    
    nfl::poly_from_modulus<T, degree, modulus> *SecretKey;

    char *Cset=getRandomBinaryBytes(size);
    char *Sset=getRandomBinaryBytes(size);
    
    char *Cout;
    uint64_t Coutlen;
    char *Sout;
    uint64_t Soutlen;
    char Intersection[size];
    
    start = std::chrono::steady_clock::now();
    
    initiateC<degree,modulus,T>(Cout,(char*)Cset,Coutlen,size,SecretKey);
    initiateS<degree,modulus,T>(Sout,Cout,(char*)Sset,Soutlen,size);
    terminateC<degree,modulus,T>(Intersection,Sout,Soutlen,size,SecretKey);
    end = std::chrono::steady_clock::now();
    
    for(int i=0;i<size;i++) {
        assert(Intersection[i]==(Cset[i]&&Sset[i]));
    }
    auto etime=get_time_us(start, end, 1);
    uint64_t bps=1000000*size/etime;
    uint64_t xfact=(Coutlen+Soutlen);
    xfact=(uint64_t)((float)xfact/(float)size);
    std::cout << "PSI : n=" << size << " \tin " << etime << " us \t\t" << 
                  bps << "bit/sec "<<
                  " \tand " << Coutlen + Soutlen << " bytes communication"<<
                  " \t expension factor = "<< xfact <<std::endl;
    
}

template<size_t degree, size_t modulus, class T>
int testSymetricPSIAsLib(uint64_t size)
{
    
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    
    nfl::poly_from_modulus<T, degree, modulus> *SecretKeyAlice;
    nfl::poly_from_modulus<T, degree, modulus> *SecretKeyBob;

    char *Aliceset=getRandomBinaryBytes(size);
    char *Bobset=getRandomBinaryBytes(size);
    
    char *Cout;
    uint64_t Coutlen;
    char *Sout;
    uint64_t Soutlen;
    char IntersectionAlice[size];
    char IntersectionBob[size];
    
    start = std::chrono::steady_clock::now();
    // Alice -> Bob
    initiateC<degree,modulus,T>(Cout,(char*)Aliceset,Coutlen,size,SecretKeyAlice);
    initiateS<degree,modulus,T>(Sout,Cout,(char*)Bobset,Soutlen,size);
    terminateC<degree,modulus,T>(IntersectionAlice,Sout,Soutlen,size,SecretKeyAlice);
    // Bob -> Alice
    initiateC<degree,modulus,T>(Cout,(char*)Bobset,Coutlen,size,SecretKeyBob);
    initiateS<degree,modulus,T>(Sout,Cout,(char*)Aliceset,Soutlen,size);
    terminateC<degree,modulus,T>(IntersectionBob,Sout,Soutlen,size,SecretKeyBob);
    end = std::chrono::steady_clock::now();
    
    for(int i=0;i<size;i++) {
        assert(IntersectionBob[i]==(Aliceset[i]&&Bobset[i]));
        assert(IntersectionAlice[i]==(Aliceset[i]&&Bobset[i]));
    }
    
    std::cout << "======================================================================" << std::endl;
    std::cout << "SPSI : n=" << size << " \tin " << get_time_us(start, end, 1) << " us \t\t" << 
                  1000000*size/get_time_us(start, end, 1) << "bit/sec "<<
                  " \tand " << (Coutlen + Soutlen)*2 << " bytes communication"<<
                  " \t expension factor = "<< (Coutlen+Soutlen)*2/size <<std::endl;
    
}

template<size_t degree, size_t modulus, class T>
int test3PartiesPSIAsLib(uint64_t size)
{
    
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    
    nfl::poly_from_modulus<T, degree, modulus> *SecretKeyAlice;

    char *Aliceset=getRandomBinaryBytes(size);
    char *Bobset=getRandomBinaryBytes(size);
    char *Charlieset=getRandomBinaryBytes(size);
    
    char *Aout;
    uint64_t Aoutlen;
    char *Bout;
    uint64_t Boutlen;
    char *Cout;
    uint64_t Coutlen;
    char IntersectionAlice[size];
    
    start = std::chrono::steady_clock::now();
    initiateC<degree,modulus,T>(Aout,(char*)Aliceset,Aoutlen,size,SecretKeyAlice);
    initiateS<degree,modulus,T>(Bout,Aout,(char*)Bobset,Boutlen,size);
    // terminateC<degree,modulus,T>(IntersectionAlice,Bout,Boutlen,size,SecretKeyAlice);
    // initiateC<degree,modulus,T>(Aout,IntersectionAlice,Aoutlen,size,SecretKeyAlice);
    //initiateS<degree,modulus,T>(Cout,Aout,(char*)Charlieset,Aoutlen,size);
    initiateS<degree,modulus,T>(Cout,Bout,(char*)Charlieset,Coutlen,size);
    terminateC<degree,modulus,T>(IntersectionAlice,Cout,Coutlen,size,SecretKeyAlice);
    end = std::chrono::steady_clock::now();
    
    for(int i=0;i<size;i++) {
        assert(IntersectionAlice[i]==(Aliceset[i]&&Bobset[i]&&Charlieset[i]));
    }
    
    std::cout << "======================================================================" << std::endl;
    std::cout << "3PSI : n=" << size << " \tin " << get_time_us(start, end, 1) << " us \t\t" << 
                  1000000*size/get_time_us(start, end, 1) << "bit/sec "<<
                  " \tand " << (Aoutlen + Boutlen + Coutlen) << " bytes communication"<<
                  " \t expension factor = "<< (Aoutlen + Boutlen + Coutlen)/size <<std::endl;
    
}
    
template<size_t degree, size_t modulus, class T>
int testPerfPSI(uint64_t size,uint64_t reps)
{
    
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    uint64_t t1 = 0;
    uint64_t t2 = 0;
    uint64_t t3 = 0;
    
    nfl::poly_from_modulus<T, degree, modulus> *SecretKey;
    
    char *Cset=getRandomBinaryBytes(size);
    char *Sset=getRandomBinaryBytes(size);
    
    char *Cout;
    uint64_t Coutlen;
    char *Sout;
    uint64_t Soutlen;
    char Intersection[size];
    for(int i=0;i<reps;i++) {
    start = std::chrono::steady_clock::now();
    initiateC<degree,modulus,T>(Cout,(char*)Cset,Coutlen,size,SecretKey);
    end = std::chrono::steady_clock::now();
    t1=t1+(get_time_us(start, end, 1));
    start = std::chrono::steady_clock::now();
    initiateS<degree,modulus,T>(Sout,Cout,(char*)Sset,Soutlen,size);
    end = std::chrono::steady_clock::now();
    t2=t2+(get_time_us(start, end, 1));
    start = std::chrono::steady_clock::now();
    terminateC<degree,modulus,T>(Intersection,Sout,Soutlen,size,SecretKey);
    end = std::chrono::steady_clock::now();
    t3=t3+(get_time_us(start, end, 1));
}
    
    std::cout << "======================================================================" << std::endl;
    std::cout << "PSI : n=" << size << " in " << (t1+t2+t3)/reps << " us " << (t1+t2+t3)/reps/size << "us/bit"<< std::endl;
    std::cout << "PSI : initC " << size << " in " << (t1)/reps << " us " << (100*t1/(t1+t2+t3)) <<"%"<< std::endl;
    std::cout << "PSI : initS " << size << " in " << (t2)/reps << " us " << (100*t2/(t1+t2+t3)) <<"%"<< std::endl;
    std::cout << "PSI : finaC " << size << " in " << (t3)/reps << " us " << (100*t3/(t1+t2+t3)) <<"%"<< std::endl;
    
    
}

void testBBF(size_t size) {
    unsigned char a[]="marco";size_t asize=5;
    unsigned char b[]="carlos";size_t bsize=6;
    unsigned char c[]="joris";size_t csize=5;
    BloomFilter BF(512,4);
    BF.insert(a,asize);
    BF.insert(b,bsize);
    bool res=BF.test(a,asize);
    assert(res==true);
    res=BF.test(b,bsize);
    assert(res==true);
    res=BF.test(c,csize);
    assert(res==false);
}

template<size_t degree, size_t modulus, class T>
void testPSIBF(size_t size) {
    /* L'idée est de :
      C crée un BF et y met son set
      C encode ce BF de 512 bytes vers un bitset de 512*8 bits
      puis intiateC et envoie à S
      S encode son BF contenant son set vers un bitset de 512*8 bits
      puis initiateS et envoie à C
      C terminateC dans un newBF
      pour chacun de ses éléments fait un newBF.test
    */

    nfl::poly_from_modulus<T, degree, modulus> *SecretKey;
    unsigned char a[]="marco";size_t asize=5;
    unsigned char b[]="carlos";size_t bsize=6;
    unsigned char c[]="joris";size_t csize=5;

    BloomFilter BF(size,4);
    BF.insert(a,asize);
    BF.insert(b,bsize); 
    BF.insert(c,bsize); 
    char* serC=BF.serialize();

    BloomFilter BFS(size,4);
    BFS.insert(a,asize);
    BFS.insert(b,bsize);    
    char* serS=BFS.serialize();

    char *Cout;
    uint64_t Coutlen;
    char *Sout;
    uint64_t Soutlen;
    char Intersection[size*8];

    initiateC<degree,modulus,T>(Cout,serC,Coutlen,size*8,SecretKey);
    initiateS<degree,modulus,T>(Sout,Cout,serS,Soutlen,size*8);
    terminateC<degree,modulus,T>(Intersection,Sout,Soutlen,size*8,SecretKey);
    
    BloomFilter BFInter(size,4);
    
    BFInter.deserialize(Intersection);
    
    bool res=BFInter.test(a,asize);
    assert(res==true);
    res=BFInter.test(b,bsize);
    assert(res==true);
    res=BFInter.test(c,csize);
    assert(res==false);
}



template<size_t degree, size_t modulus, class T>
int run()
{
    
    
    testPerfPSI<degree,modulus,T>(10,2);
    
    
    testBBF(512);
    std::cout<<"---"<<std::endl;
    testPSIBF<degree,modulus,T>(512);
    
    
    return 0;
}



template<size_t degree, size_t modulus, class T>
int launch() {
    Transport::GraphFactory gf("/vagrant/ubimob/graph.txt-dump", false);
    const Transport::Graph * g = gf.get();
    gf.setAll2();
    //gf.PickUpByRegion(43.5115, 1.3177, 43.6862, 1.5573, 10, 100 );
    std::cout << g->PickUpZone().size() << std::endl;
    typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
    boost::char_separator<char> sep(";");
    string filename="/vagrant/ubimob/analysis/experiments/dec2015/my_instances_16-12-2015.csv";
    ifstream in(filename.c_str());
    if (!in.is_open()) exit(0);
    vector< string > vec;
    string line;
    int i=0;
    int car_1, car_2, foot_1, foot_2;
    int iso=3600;
    int step=60;
    int ratio=1;
    Privacy::Toolbox * tb=new Privacy::Toolbox();
    tb->create_result("/vagrant/ubimob/analysis/experiments/dec2015/astar.csv");
    while (getline(in,line)){
      Tokenizer tok(line,sep);
      vec.assign(tok.begin(),tok.end());
      if(i>0){
//###############################################################################################################
        car_1=std::atoi(vec.at(0).c_str());
        foot_1=std::atoi(vec.at(1).c_str());
        car_2=std::atoi(vec.at(2).c_str());
        foot_2=std::atoi(vec.at(3).c_str());
        auto start = std::chrono::steady_clock::now();
        auto end = std::chrono::steady_clock::now();
        Privacy::Driver * d1=new Privacy::Driver(car_1, car_2);
        Privacy::Pedestrian * r1=new Privacy::Pedestrian(foot_1, foot_2);
        Privacy::Manager * m=new Privacy::Manager();
        Privacy::Output output;
        //---record scenario id, step and isochrone radius
        output.scenario_id = i;
        output.isochrone_radius = iso;
        output.step = step;
        uint64_t size = g->num_vertices();

        start = std::chrono::steady_clock::now();
        d1->findRoad(g);
        end = std::chrono::steady_clock::now();
        //---record driver origin isochrone running time
        output.driver_origin_isochrone_time = boost::lexical_cast<double>(get_time_us(start, end, 1)/1000000);
        //---record driver destination isochrone running time
        output.driver_destination_isochrone_time = boost::lexical_cast<double>(get_time_us(start, end, 1)/1000000);

        start = std::chrono::steady_clock::now();
        r1->findPickup(g, iso);
        end = std::chrono::steady_clock::now();
        //---record rider origin isochrone running time
        output.rider_origin_isochrone_time = boost::lexical_cast<double>(get_time_us(start, end, 1)/1000000);

        start = std::chrono::steady_clock::now();
        r1->findDropoff(g, iso);
        end = std::chrono::steady_clock::now();
        //---record rider destination isochrone running time
        output.rider_destination_isochrone_time = boost::lexical_cast<double>(get_time_us(start, end, 1)/1000000);


        int StarUp = 0;
        int EndUp = step;
        int StarOff = 0;
        int EndOff = step;
        NodeList pickUp, dropOff;
        int num_steps_pickup = 0, num_steps_dropoff=0;
        // §§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§------Pickup selection------------§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§
        start = std::chrono::steady_clock::now();
         while (EndUp <= iso){
            d1->choosePickup(StarUp/ratio, EndUp/ratio);
            r1->choosePickup(StarUp, EndUp);
            NodeList pickup_driver_set = d1->getPickup();
            NodeList pickup_rider_set = r1->getPickup();
            uint8_t DriverPickupSet[size], RiderPickupSet[size];
            for(uint64_t i=0;i<size;i++) {
                DriverPickupSet[i]=0;
                RiderPickupSet[i]=0;
            }
            //-----------------Driver
            BOOST_FOREACH(int node, pickup_driver_set){
                DriverPickupSet[node]=1;
            }
            //-----------------Pedestrian
            BOOST_FOREACH(int node, pickup_rider_set){
                RiderPickupSet[node]=1;
            }
            nfl::poly_from_modulus<T, degree, modulus> *SecretKey;
            // ################################## PickUp PSI @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
            char *DriverPickupOut;
            uint64_t DriverPickupOutLen;
            initiateC<degree,modulus,T>(DriverPickupOut,(char*)DriverPickupSet,DriverPickupOutLen,size,SecretKey);
            char *RiderPickupOut;
            uint64_t RiderPickupOutLen;
            initiateS<degree,modulus,T>(RiderPickupOut,DriverPickupOut,(char*)RiderPickupSet,RiderPickupOutLen,size);
            char PSI_Pickup[size];
            terminateC<degree,modulus,T>(PSI_Pickup,RiderPickupOut,RiderPickupOutLen,size,SecretKey);
            pickUp.clear();
            for(int i=0;i<size;i++){
                if(PSI_Pickup[i]==1){
                    pickUp.push_back(i);
                }
            }
            //StarUp = EndUp;
            EndUp += step;
            num_steps_pickup++;

            if (pickUp.size() > 0){
                break;
            }
         }
         end = std::chrono::steady_clock::now();
         //---record psi pickup runtime
         output.psi_pickup_time = boost::lexical_cast<double>(get_time_us(start, end, 1)/1000000);
         output.num_steps_pickup = num_steps_pickup;

         start = std::chrono::steady_clock::now();
        // §§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§------Dropoff selection------------§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§
         while (EndOff <= iso){
            d1->chooseDropoff(StarOff/ratio, EndOff/ratio);
            r1->chooseDropoff(StarOff, EndOff);
            NodeList dropoff_driver_set = d1->getDropoff();
            NodeList dropoff_rider_set = r1->getDropoff();
            uint8_t DriverDropoffSet[size], RiderDropoffSet[size];
            for(uint64_t i=0;i<size;i++) {
                DriverDropoffSet[i]=0;
                RiderDropoffSet[i]=0;
            }
            //-----------------Driver
            BOOST_FOREACH(int node, dropoff_driver_set){
                DriverDropoffSet[node]=1;
            }
            //-----------------Pedestrian
            BOOST_FOREACH(int node, dropoff_rider_set){
                RiderDropoffSet[node]=1;
            }
            nfl::poly_from_modulus<T, degree, modulus> *SecretKey;
            // ################################## DropOff PSI @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
            char *DriverDropoffOut;
            uint64_t DriverDropoffOutLen;
            initiateC<degree,modulus,T>(DriverDropoffOut,(char*)DriverDropoffSet,DriverDropoffOutLen,size,SecretKey);
            char *RiderDropoffOut;
            uint64_t RiderDropoffOutLen;
            initiateS<degree,modulus,T>(RiderDropoffOut,DriverDropoffOut,(char*)RiderDropoffSet,RiderDropoffOutLen,size);
            char PSI_Dropoff[size];
            terminateC<degree,modulus,T>(PSI_Dropoff,RiderDropoffOut,RiderDropoffOutLen,size,SecretKey);
            dropOff.clear();
            for(int i=0;i<size;i++){
                if(PSI_Dropoff[i]==1){
                    dropOff.push_back(i);
                }
            }
            //StarOff = EndOff;
            EndOff += step;
            num_steps_dropoff++;

            if (dropOff.size() > 0){
                break;
            }
         }
         end = std::chrono::steady_clock::now();
        //---record psi dropoff runtime
         output.psi_dropoff_time = boost::lexical_cast<double>(get_time_us(start, end, 1)/1000000);
         output.num_steps_dropoff = num_steps_dropoff;


         auto psi_result = m->getPSI(pickUp, dropOff);
         
         
         output.pickup_size = psi_result.PickUp.size();
         output.dropoff_size = psi_result.DropOff.size();


        start = std::chrono::steady_clock::now();
        m->GetAllPath(g, psi_result);
        end = std::chrono::steady_clock::now();
        //---record shared path computation runtime
        output.shared_path_computing_time = boost::lexical_cast<double>(get_time_us(start, end, 1)/1000000);

        if (m->shared_path_len() > 0){

            start = std::chrono::steady_clock::now();
            d1->getFavorites(m->shared_path);
            end = std::chrono::steady_clock::now();
            output.driver_path_ordering_time = boost::lexical_cast<double>(get_time_us(start, end, 1)/1000000);


            start = std::chrono::steady_clock::now();
            r1->getFavorites(m->shared_path);
            end = std::chrono::steady_clock::now();
            output.rider_path_ordering_time = boost::lexical_cast<double>(get_time_us(start, end, 1)/1000000);

            start = std::chrono::steady_clock::now();
            m->match(*d1, *r1);
            end = std::chrono::steady_clock::now();
            output.path_election_time = boost::lexical_cast<double>(get_time_us(start, end, 1)/1000000);

            //id of positions
            output.driver_origin = d1->posStart;
            output.driver_destination = d1->posEnd;
            output.rider_origin = r1->posStart;
            output.rider_destination = r1->posEnd;

            //id of pickup/dropoff
            output.privacy_pickup = m->getThePath().start;
            output.privacy_dropoff = m->getThePath().end;
            //postions of the distributed solution
            output.cd_positions = tb->cd_carpooling_positions(*d1, *r1, *m, g);
            //costs of the distributed solution
            output.cd_costs = tb->cd_costs(*d1, *r1, *m, g);
            //size of potential pickup for pedesdrian
            output.rider_pickup_size = tb->dlen(r1->data_before);
            //size of potential pickup for driver
            output.driver_pickup_size = tb->dlen(d1->data_before);
            //size of potential dropoff for pedesdrian
            output.rider_dropoff_size = tb->dlen(r1->data_after);
            //size of potential dropoff for driver
            output.driver_dropoff_size = tb->dlen(d1->data_after);

            output.shared_path_size = m->shared_path_len();
            //centralized test
            Privacy::cc_output cc;
            cc=tb->cc_carpooling_test(*d1,*r1,g);
            //costs in centralized
            output.cc_costs=cc.cc_costs;
            //pickup in centralized
            output.optimal_pickup=cc.cc_pickup;
            //dropoff in centralized
            output.optimal_dropoff=cc.cc_dropoff;
            //position in centralized
            output.cc_positions=cc.cc_positions;
            //runtime in centralized
            output.optimal_runtime=boost::lexical_cast<double>(cc.cc_time/1000);

            output.driver_original_trip_cost = tb->pointTopoint(g, car_1, car_2, true);
            output.rider_original_trip_cost = tb->pointTopoint(g, foot_1, foot_2, false);

            
            //output.cc_detour=tb->cc_detour(*d1, g, cc );
            //output.cd_detour=tb->cd_detour(*d1, g, output.cd_costs);

            tb->save_result("/vagrant/ubimob/analysis/experiments/dec2015/astar.csv", output, g);
            std::cout << "======================================================================> solution " << i << std::endl;
        }
//###############################################################################################################
    }
    i++;
}
    return 0;
}

int main(int argc, char const *argv[])
{
    return launch<CONFIG>();
}


