/*
 * fileio.h
 *
 *  Created on: 2019. 5. 27.
 *      Author: keti-hajun
 */

#ifndef FILEIO_H_
#define FILEIO_H_

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <stdio.h>
#include <fstream>

using namespace std;

// Text file data load
void load_data(string file_name, vector<double> *data, string delimiter);
void load_data(string file_name, vector<float> *data);

#endif /* FILEIO_H_ */
