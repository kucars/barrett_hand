#include "MacJointTraj.hh"
#include <stdio.h>
#include <string>


int main() {
  FILE *f = fopen("owd_abort1.bin","r");
  if (!f) {
    printf("Could not open file\n");
    return -1;
  }
  std::string data;
  char c;
  while (fread(&c,1,1,f)) {
    data.push_back(c);
  };
  printf("Read %zd bytes\n",data.size());
  try {
    BinaryData bd(data);
    OWD::MacJointTraj mytraj(bd);
    printf("Extracted a traj with duration %f\n",mytraj.duration);
  } catch (const char *err) {
    printf("Error while recreating MacJointTraj: %s\n",err);
  }
  fclose(f);
}
