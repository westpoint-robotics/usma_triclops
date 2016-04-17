#include <iostream>
#include "usma_triclops/bumblebeecamera.h"

using namespace std;

int main(int argc, char *argv[])
{
    BumbleBeeCamera bb2;
    bb2.startCamera();
    bb2.retrieveImages();


    cout << "Hello World!" << endl;
    bb2.shutdown();
    return 0;
}
