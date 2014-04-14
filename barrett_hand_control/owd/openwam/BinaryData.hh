// FILENAME = BinaryData.h

#ifndef binarydata_h
#define binarydata_h

#include <string>
#include <ostream>
#include <istream>
#include <vector>


enum BinaryDataType {
  BDT_Bool,
  BDT_Char,
  BDT_Short,
  BDT_Int,
  BDT_Long,
  BDT_Record,
  BDT_NetworkID,
  BDT_BinaryData,
  BDT_UUID,
  BDT_String,
  BDT_Double,
  BDT_DoubleVector
};

class BinaryData : public std::string {


protected:
  void put_raw_data(
                    const void *d,
                    int s) ;

  void get_raw_data(
                    void *d,
                    int s) const;

  void put_type(
                BinaryDataType bdt);

  void get_and_check_type(
                          BinaryDataType bdt) const;

public: 
	
  BinaryData(const BinaryData& b);
  BinaryData();
  explicit BinaryData(const void *v, size_t s);
  BinaryData(const std::string &v);
  ~BinaryData();
  
  void reset();						// Prepare the BinaryData to accept new objects, overwriting the old ones
  void rewind() { nextg = 0; }	// Rewind the "get" pointer so that the objects can be retrieved again
  
  // Storage of ints
  void PutInt(const int toStore);
  int GetInt() const;

  // Storage of bools
  void PutChar(const char toStore);
  char GetChar() const;

  // Storage of bools
  void PutBoolean(const bool toStore);
  bool GetBoolean() const;

  // Storage of shorts
  void PutShort(const short toStore);
  short GetShort() const;
	
  // Storage of longs
  void PutLong(const long toStore);
  long GetLong() const;

  // Storage of doubles
  void PutDouble(const double toStore);
  double GetDouble() const;

  void PutString(const std::string& toStore);
  std::string GetString() const;

  void PutDoubleVector(const std::vector<double> &toStore);
  std::vector<double> GetDoubleVector() const;

  std::string BDT_Type_String(BinaryDataType bt) const;
  
protected:
  mutable int nextg;  // Position of next "get" operation

};


#endif
