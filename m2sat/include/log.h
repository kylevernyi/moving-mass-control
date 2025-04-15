#pragma once
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <fstream>
#include <sstream>
#include <ctime>
#include <Eigen/Dense>
#include <chrono> 

#include "telemetry.h"

using namespace Eigen;

void writeHeader(std::ofstream& file);
void saveData(std::ofstream& file, MatrixXd  matrix);
std::string generateTimestampedFilename();
uint64_t getTimestamp();
void LogTele(std::ofstream& file, const telemetry_t & tele);