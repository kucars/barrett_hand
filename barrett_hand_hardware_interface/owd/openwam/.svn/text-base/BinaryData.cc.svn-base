#include <assert.h>
#include <stdio.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include "BinaryData.hh"


/****************************************************************** 
 * Record Functions                                               *
 ******************************************************************/

/***************************************
  BinaryData functions
 ***************************************/

BinaryData::BinaryData()
	:
	nextg(0)
{ 
}

BinaryData::BinaryData(const std::string &s) 
	: 
  std::string(s),
  nextg(0)
{
}

BinaryData::BinaryData(const BinaryData& bd) 
	:
  std::string(bd),
  nextg(bd.nextg)
{
}
	

BinaryData::~BinaryData() {
}


void BinaryData::reset() {
  resize(0);
  nextg=0;
}


void BinaryData::PutInt(const int toStore) {
  int netint = htonl(toStore);
  put_type(BDT_Int);
  put_raw_data(&netint,sizeof(int));
}

int BinaryData::GetInt() const{
  get_and_check_type(BDT_Int);
  int netint;
  get_raw_data(&netint,sizeof(int));
  return(ntohl(netint));
}

void BinaryData::PutShort(const short toStore) {
  short netshort = htons(toStore);
  put_type(BDT_Short);
  put_raw_data(&netshort,sizeof(short));
}

short BinaryData::GetShort() const {
  get_and_check_type(BDT_Short);
  short netshort;
  get_raw_data(&netshort,sizeof(short));
  return(ntohs(netshort));
}

void BinaryData::PutBoolean(const bool toStore) {
  put_type(BDT_Bool);
  put_raw_data(&toStore,sizeof(bool));
}

bool BinaryData::GetBoolean() const {
  get_and_check_type(BDT_Bool);
  bool netshort;
  get_raw_data(&netshort,sizeof(bool));
  return(netshort);
}

void BinaryData::PutChar(const char toStore) {
  put_type(BDT_Char);
  put_raw_data(&toStore,sizeof(char));
}

char BinaryData::GetChar() const {
  get_and_check_type(BDT_Char);
  short netshort;
  get_raw_data(&netshort,sizeof(char));
  return(netshort);
}

void BinaryData::PutDouble(const double toStore) {
  put_type(BDT_Double);
  put_raw_data(&toStore,sizeof(double));
}

double BinaryData::GetDouble() const {
  get_and_check_type(BDT_Double);
  double netdouble;
  get_raw_data(&netdouble,sizeof(double));
  return(netdouble);
}

void BinaryData::PutLong(const long toStore) {
  put_type(BDT_Long);
  long netlong = htonl(toStore);
  put_raw_data(&netlong, sizeof(long));
}

long BinaryData::GetLong() const {
  get_and_check_type(BDT_Long);
  long netlong;
  get_raw_data(&netlong, sizeof(long));
  return(ntohl(netlong));
}


void BinaryData::PutString(const std::string& toStore) {
  unsigned int stringsize = toStore.size();
  put_type(BDT_String);
  PutInt(stringsize);
  put_raw_data(toStore.c_str(),stringsize);
}

std::string BinaryData::GetString() const {
  get_and_check_type(BDT_String);
  unsigned int stringsize = GetInt();
  std::string s;
  char c;
  for (unsigned int i=0; i<stringsize; ++i) {
    get_raw_data(&c,1);
    s.push_back(c);
  }
  return(s);
}

void BinaryData::PutDoubleVector(const std::vector<double> &toStore) {
  unsigned int vectorsize = toStore.size();
  put_type(BDT_DoubleVector);
  PutInt(vectorsize);
  put_raw_data(&toStore[0],vectorsize*sizeof(double));
}

std::vector<double> BinaryData::GetDoubleVector() const {
  get_and_check_type(BDT_DoubleVector);
  unsigned int vectorsize = GetInt();
  std::vector<double> v(vectorsize);
  get_raw_data(&v[0],vectorsize*sizeof(double));
  return v;
}

void BinaryData::put_type(const BinaryDataType bt)
{
  unsigned char s = bt;  // store the type in a single byte
  put_raw_data(&s, sizeof(unsigned char));
}
            
void BinaryData::get_and_check_type(const BinaryDataType bt) const
{
  BinaryDataType s(BDT_Bool);  // initialize to something so that upper bytes
                               // are set to zero
  get_raw_data(&s, sizeof(unsigned char));
  printf("Looking for type %s\n",BDT_Type_String(bt).c_str());
  if(s != bt) {
    char errmsg[200];
    snprintf(errmsg,200,"Mismatched data records: expected type %s (value %d) but found type %s (value %d)",
             BDT_Type_String(bt).c_str(),
	     bt,
             BDT_Type_String(s).c_str(),
	     s);
    printf("BinaryData error: %s\n",errmsg);
    throw errmsg;
  }
}        

void BinaryData::put_raw_data(const void *d, int s) 
{
  for (int i=0; i<s; ++i) {
    push_back( ((char *)d)[i]);
  }
}


void BinaryData::get_raw_data(void *d, int s) const 
{
  if ((unsigned int)(nextg+s) > size()) {
    throw "Attempted read past end of data";
  }
  for (int i=0; i<s; ++i) {
    ((char *)d)[i] = operator[](nextg++);
  }
}

std::string BinaryData::BDT_Type_String(BinaryDataType bt) const {
  switch (bt) {
  case BDT_Bool:
    return std::string("Bool");
  case BDT_Char:
    return std::string("Char");
  case BDT_Short:
    return std::string("Short");
  case BDT_Int:
    return std::string("Int");
  case BDT_Long:
    return std::string("Long");
  case BDT_Record:
    return std::string("Record");
  case BDT_NetworkID:
    return std::string("NetworkID");
  case BDT_BinaryData:
    return std::string("BinaryData");
  case BDT_UUID:
    return std::string("UUID");
  case BDT_String:
    return std::string("String");
  case BDT_Double:
    return std::string("Double");
  case BDT_DoubleVector:
    return std::string("DoubleVector");
  default:
    return std::string("Undefined");
  }
}
