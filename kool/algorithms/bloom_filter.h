#include "keccak.h"
#include "/home/vagrant/nfllib/include/nfl.hpp"

class BloomFilter {
private:
	size_t nbkeys;
	unsigned char *BF;
	unsigned char *keys;
	size_t size; // in bytes
	

public:
	BloomFilter(size_t const sByte, size_t nkeys) {
		size=sByte; nbkeys=nkeys;		
		fastrandombytes(keys=(unsigned char*)(malloc(size*nbkeys)),size*nbkeys);
		BF=(unsigned char*)memset(malloc(size),0,size);
		
	}
	
	void insert(unsigned char* element,size_t el_sizeByte) {
		insert(element, el_sizeByte,BF);
	}

	bool test(unsigned char* element,size_t el_sizeByte) {
		unsigned char digest[64];
		unsigned char kdigests[64];
		unsigned char* emptyBF=(unsigned char*)memset(malloc(size/8),0,size/8);
		unsigned char* res=(unsigned char*)memset(malloc(size/8),0,size/8);
		insert(element, el_sizeByte,emptyBF);	
		And(res,emptyBF,BF,size);
		return Equal(res,emptyBF,size);
	}
	
	private :
	
	void insert(unsigned char* element,size_t el_sizeByte, unsigned char *filter) {
		unsigned char digest[size];
		unsigned char kdigests[size];
		MOK_SHA((uint8_t*)element,el_sizeByte,(uint8_t*)digest);
		for(int j=0;j<nbkeys;j++) {
			Xor(kdigests,digest,(keys+j*size),size);
			Or(filter,filter,digest,size);
		}
	}
	
static void debugmsg(const char *head,const unsigned char *buf,const size_t size) {
	uint64_t *ubuf=(uint64_t*)buf;
	std::cout << head << " \t0x"; for(int tr=0;tr<size/8/8;tr++) std::cout << boost::format("%016x ") % (uint64_t)(*(ubuf+tr)); std::cout<< std::endl;
	
}

 void Xor (unsigned char*out,unsigned char const* a,unsigned char const* b,size_t const size) {
	for (int i=0;i<size/8;i++) {
		out[i]=a[i]^b[i];
	}
}

 void Or (unsigned char*out,unsigned char const* a,unsigned char const* b,size_t const size) {
	for (int i=0;i<size/8;i++) {
		out[i]=a[i]|b[i];
	}
}

 void And (unsigned char*out,unsigned char const* a,unsigned char const* b,size_t const size) {
	for (int i=0;i<size/8;i++) {
		out[i]=a[i]&b[i];
	}

}

 bool Equal (unsigned char const* a,unsigned char const* b,size_t const size) {
	for (int i=0;i<size/8;i++)
		if(a[i]!=b[i]) return false;
	return true;
}

unsigned char masks[8]={(char)1,(char)2,(char)4,(char)8,(char)16,(char)32,(char)64,(char)128};

public:
char *serialize() {	
	char *res=(char*)malloc(size*8);
	for (int i=0;i<size;i++) 
		for(int bit=0;bit<8;bit++)
			res[8*i+bit]=(BF[i]&masks[bit])>>bit;
	return res;
}

void deserialize(char* in) {
	for (int i=0;i<size;i++) {
		BF[i]=0;
		for(int bit=0;bit<8;bit++)
			BF[i]+=in[8*i+bit]*masks[bit];
	}
	
}

};

