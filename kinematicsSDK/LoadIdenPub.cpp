#include "LoadIdenPub.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <deque>
#include <cmath>
#include <cstring>

using namespace std;

typedef struct {
	double torLoad[7];
	double torNoLoad[7];
}LoadTor;

typedef struct {
	double m;
	double mr[3];
	double inertia[6];
}LoadDynamic;

static vector<string> strSplite(const string& str, const char& del)
{
	stringstream ss(str);
	string tem;
	vector<string> vecItem;
	while (getline(ss, tem, del)) {
		vecItem.push_back(tem);
#ifdef G_Debug
		cout << tem << endl;
#endif
	}
	return vecItem;
}

static bool strSpliteSRS(const string& str, vector<vector<string>>& loadData)
{
	deque<string> deqItem;
	stringstream ss(str);
	string tem;
	const size_t dataCloNum = 7;
	int dataPos = 0;
	while (getline(ss, tem, ',')) {
		dataPos++;

		if (4 == dataPos ||
			6 == dataPos ||
			7 == dataPos ||
			11 == dataPos ||
			13 == dataPos ||
			14 == dataPos
			) {
			deqItem.push_back(tem);
		}
		if (15 == dataPos) {
			deqItem.push_front(tem);
			dataPos = 0;
		}
	}
	if (deqItem.size() < dataCloNum || loadData.size() < dataCloNum) {
		cout << "ERROR:data must be 7  columns" << endl;
		return false;
	}
	for (int i = 0; i < loadData.size(); i++) {
		loadData.at(i).push_back(deqItem.at(i));
	}
	return true;
}

LoadIdenErrCode OnCalLoadTor_7_MarvinSRS(LoadTor* tor, int isNoLoad, const char *userPath)
{
	if (tor == NULL) {
		printf("tor is null!\n");
		return LOAD_IDEN_CalErr;
	}
	string strUserPath(userPath);
	string strLDFileName("/LoadData.csv");
	string strNoLDFileName("/NoLoadData.csv");
	string tmpPath;
	if (1 == isNoLoad) {
		printf("Calculating no-load data...\n");
		tmpPath = strUserPath + strNoLDFileName;
	}
	else {
		tmpPath = strUserPath + strLDFileName;
	}
	ifstream ifLoadFile(tmpPath, ios::in);
	if (!ifLoadFile.is_open()) {
		cout << "ERROR: file load failed, pls check file!" << endl;
		return LOAD_IDEN_OpenSmpDateFieErr;
	}
	string strLoadDataLine;
	vector<vector<string>> vecLoadData(7);
	while (getline(ifLoadFile, strLoadDataLine)) {
		if (strSpliteSRS(strLoadDataLine, vecLoadData) == false) {
			cout << "ERROR: parse file failed, pls check file!" << endl;
			return LOAD_IDEN_OpenSmpDateFieErr;
		}
	}
	ifLoadFile.close();
	enum LoadDataType {
		DataTag,
		Axis4Pos,
		Axis6Pos,
		Axis7Pos,
		Axis4Tor,
		Axis6Tor,
		Axis7Tor
	};
	string strIdenCfgName(strUserPath + "/CfgFile/LoadIdenCfg_Marvin_SRS.txt");

	ifstream ifsConFile(strIdenCfgName, ios::in);
	if (!ifsConFile.is_open()) {
		cout << "ERROR: load cfg file failed, pls check file<LoadIdenCfg_Marvin_SRS.txt>" << endl;
		return LOAD_IDEN_OpenCfgFileErr;
	}
	string strCfg;
	if (!getline(ifsConFile, strCfg)) {
		cout << "Error: load config failed, type-1, pls contact supplier for help." << endl;
		return LOAD_IDEN_OpenCfgFileErr;
	}
	ifsConFile.close();
	vector<string> vecCon;
	vecCon = strSplite(strCfg, ',');
	if (vecCon.size() != 17) {
		cout << "Error: load config failed, type-2, pls contact supplier for help." << endl;
		return LOAD_IDEN_OpenCfgFileErr;
	}
	const double torTag[7] = { 0, 1111, 2222, 3333, 4444, 5555, 6666 };
    double trajCondation[10] = { 1, 0, 1, 0, 5, 4, 95, 94, 20, -20 };
	double sampleCy = 1000;

	int dataTagPos = 0;
	vector<double> vecTor;
	double dtemp = 0;
	int i;
	int LoadDataNum = static_cast<int>(vecLoadData.at(DataTag).size());
	int trajCalTimes = 0;
	double tor7[7] = { 0 };
	int dataFlagCount = 0;
	vecTor.clear();
	for (i = dataTagPos; i < LoadDataNum; i++) {
		if (torTag[1] == stod(vecLoadData.at(DataTag).at(i))) {
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(Axis4Pos).at(i));
				if (dtemp<trajCondation[0] && dtemp>trajCondation[1]) {
					break;
				}
				i++;
			}
			trajCalTimes++;
			while (i < LoadDataNum) {
				dtemp = stod(vecLoadData.at(Axis4Pos).at(i));
				if (dtemp<trajCondation[0] && dtemp>trajCondation[1]) {
					vecTor.push_back(stod(vecLoadData.at(Axis4Tor).at(i)));
					i++;
				}
				else {
					dataTagPos = i;
					break;
				}

			}
		}
		if (2 == trajCalTimes) {
			trajCalTimes = 0;
			dataFlagCount++;
			break;
		}
	}
	dtemp = 0;
	for (int num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[0] = dtemp / vecTor.size();
	vecTor.clear();
	for (i = dataTagPos; i < LoadDataNum; i++) {
		if (torTag[2] == stod(vecLoadData.at(DataTag).at(i))) {
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(Axis6Pos).at(i));
				if (dtemp<trajCondation[2] && dtemp>trajCondation[3]) {
					break;
				}
				i++;
			}
			trajCalTimes++;
			while (i < LoadDataNum) {
				dtemp = stod(vecLoadData.at(Axis6Pos).at(i));
				if (dtemp<trajCondation[2] && dtemp>trajCondation[3]) {
					vecTor.push_back(stod(vecLoadData.at(Axis6Tor).at(i)));
					i++;
				}
				else {
					dataTagPos = i;
					break;
				}
			}
		}
		if (2 == trajCalTimes) {
			trajCalTimes = 0;
			dataFlagCount++;
			break;
		}
	}
	dtemp = 0;
	for (int num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[1] = dtemp / vecTor.size();
	vecTor.clear();
	for (i = dataTagPos; i < LoadDataNum; i++) {
		if (torTag[3] == stod(vecLoadData.at(DataTag).at(i))) {
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(Axis7Pos).at(i));
				if (dtemp<trajCondation[4] && dtemp>trajCondation[5]) {
					break;
				}
				i++;
			}
			trajCalTimes++;
			while (i < LoadDataNum) {
				dtemp = stod(vecLoadData.at(Axis7Pos).at(i));
				if (dtemp<trajCondation[4] && dtemp>trajCondation[5]) {
					vecTor.push_back(stod(vecLoadData.at(Axis7Tor).at(i)));
					i++;
				}
				else {
					dataTagPos = i;
					break;
				}
			}
		}
		if (2 == trajCalTimes) {
			trajCalTimes = 0;
			dataFlagCount++;
			break;
		}
	}
	dtemp = 0;
	for (int num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[2] = dtemp / vecTor.size();
	vecTor.clear();
	for (i = dataTagPos; i < LoadDataNum; i++) {
		if (torTag[3] == stod(vecLoadData.at(DataTag).at(i))) {
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(Axis7Pos).at(i));
				if (dtemp<trajCondation[6] && dtemp>trajCondation[7]) {
					break;
				}
				i++;
			}
			trajCalTimes++;
			while (i < LoadDataNum) {
				dtemp = stod(vecLoadData.at(Axis7Pos).at(i));
				if (dtemp<trajCondation[6] && dtemp>trajCondation[7]) {
					vecTor.push_back(stod(vecLoadData.at(Axis7Tor).at(i)));
					i++;
				}
				else {
					dataTagPos = i;
					break;
				}
			}
		}
		if (2 == trajCalTimes) {
			trajCalTimes = 0;
			dataFlagCount++;
			break;
		}
	}
	dtemp = 0;
	for (int num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[3] = dtemp / vecTor.size();
	double dtemp2 = 0;
	bool inrange = false;
	vecTor.clear();
	inrange = false;
	for (i = dataTagPos; i < LoadDataNum - 1; i++) {
		if (torTag[6] == stod(vecLoadData.at(DataTag).at(i))) {
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(Axis7Pos).at(i));
				dtemp2 = stod(vecLoadData.at(Axis7Pos).at(i + 1));
				if (((dtemp2 - dtemp) * sampleCy < trajCondation[8]) && (dtemp2 - dtemp) * sampleCy > trajCondation[9]) {
					break;
				}
				i++;
			}
			//
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(Axis7Pos).at(i));
				dtemp2 = stod(vecLoadData.at(Axis7Pos).at(i + 1));
				if (((dtemp2 - dtemp) * sampleCy < trajCondation[8]) && (dtemp2 - dtemp) * sampleCy > trajCondation[9]) {
					vecTor.push_back(stod(vecLoadData.at(Axis7Tor).at(i)));
					inrange = true;
				}
				else {
					dataTagPos = i;
					break;
				}
				i++;
			}
		}
		if (inrange) {
			dataTagPos = i;
			dataFlagCount++;
			break;
		}
	}
	dtemp = 0;
	for (int num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[6] = dtemp / vecTor.size();
	vecTor.clear();
	inrange = false;
	for (i = dataTagPos; i < LoadDataNum - 1; i++) {

		if (torTag[5] == stod(vecLoadData.at(DataTag).at(i))) {
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(Axis6Pos).at(i));
				dtemp2 = stod(vecLoadData.at(Axis6Pos).at(i + 1));
				if (((dtemp2 - dtemp) * sampleCy < trajCondation[8]) && (dtemp2 - dtemp) * sampleCy > trajCondation[9]) {
					break;
				}
				i++;
			}
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(Axis6Pos).at(i));
				dtemp2 = stod(vecLoadData.at(Axis6Pos).at(i + 1));
				if (((dtemp2 - dtemp) * sampleCy < trajCondation[8]) && (dtemp2 - dtemp) * sampleCy > trajCondation[9]) {
					vecTor.push_back(stod(vecLoadData.at(Axis6Tor).at(i)));
					inrange = true;
				}
				else {
					dataTagPos = i;
					dataFlagCount++;
					break;
				}
				i++;
			}
		}
		if (inrange) {
			dataTagPos = i;
			break;
		}
	}
	dtemp = 0;
	for (int num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[5] = dtemp / vecTor.size();
	inrange = false;
	vecTor.clear();
	for (i = dataTagPos; i < LoadDataNum - 1; i++) {
		if (torTag[4] == stod(vecLoadData.at(DataTag).at(i))) {
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(Axis6Pos).at(i));
				dtemp2 = stod(vecLoadData.at(Axis6Pos).at(i + 1));
				if (((dtemp2 - dtemp) * sampleCy < trajCondation[8]) && (dtemp2 - dtemp) * sampleCy > trajCondation[9]) {
					break;
				}
				i++;
			}
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(Axis6Pos).at(i));
				dtemp2 = stod(vecLoadData.at(Axis6Pos).at(i + 1));
				if (((dtemp2 - dtemp) * sampleCy < trajCondation[8]) && (dtemp2 - dtemp) * sampleCy > trajCondation[9]) {
					vecTor.push_back(stod(vecLoadData.at(Axis6Tor).at(i)));
					inrange = true;
				}
				else {
					dataTagPos = i;
					break;
				}
				i++;
			}
		}
		if (inrange) {
			dataTagPos = i;
			dataFlagCount++;
			break;
		}
	}
	dtemp = 0;
	if (dataFlagCount != 7) {
		cout << "ERROR: insufficient effective data, pls check LoadData.csv or NoLoadData.csv" << endl;
		return LOAD_IDEN_DataSmpErr;
	}
	for (int num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[4] = dtemp / vecTor.size();
	for (int i = 0; i < 7; i++) {
		tor->torNoLoad[i] = stod(vecCon.at(i + 10));
		tor->torLoad[i] = tor7[i];
	}
	if (1 == isNoLoad) {
		for (int i = 0; i < 7; i++) {
			tor->torNoLoad[i] = tor7[i];
			vecCon.at(i + 10) = to_string(tor7[i]);
			tor->torLoad[i] = 0;
		}
		ofstream ofsConFile(strIdenCfgName);
		if (!ofsConFile.is_open()) {
			cout << "ERROR: pls check ./LoadData/CfgFileLoadIdenCfg" << endl;
			return LOAD_IDEN_OpenCfgFileErr;
		}
		for (int i = 0; i < vecCon.size() - 1; i++) {
			if (!(ofsConFile << vecCon.at(i) << ",")) {
				cout << "ERROR: pls check ./LoadData/CfgFileLoadIdenCfg" << endl;
				return LOAD_IDEN_OpenCfgFileErr;
			}
		}
		ofsConFile << vecCon.at(vecCon.size() - 1) << "\n";
		ofsConFile.close();
		cout << "Set no-load parameters finished" << endl;
	}
	return LOAD_IDEN_NoErr;
}

LoadIdenErrCode OnCalLoadDynamic_7_MarvinSRS(LoadDynamic* loadDyn, const char *userPath)
{
	cout << "Calculating load data..." << endl;
	LoadTor loadTor;
	memset(&loadTor, 0, sizeof(loadTor));

	LoadIdenErrCode retCode = LOAD_IDEN_NoErr;
	retCode = OnCalLoadTor_7_MarvinSRS(&loadTor, 0, userPath);
	if (retCode != LOAD_IDEN_NoErr) {
		return retCode;
	}

	// if (OnCalLoadTor_7_MarvinSRS(&loadTor, 0, userPath) == false) {
	// 	printf("failed to cal load tor\n");
	// 	return false;
	// }
	double g = 9.8, pi = 3.1415926;
	double acc = 1000.0 / 180 * pi;
	double l4 = 280.0 / 1000.0;
	double l7 = 160.0 / 1000.0;
	double tor4, tor6;
	const double calErr = 0;
	tor4 = (loadTor.torLoad[0] - loadTor.torNoLoad[0]) + calErr;
	tor6 = (loadTor.torLoad[1] - loadTor.torNoLoad[1]);
	double Me, me, le, z;
	Me = (fabs(tor4) - fabs(tor6)) / l4;
	me = Me / g;
	if (me < 0.001 || me > 50 || isnan(me)) {
		cout << "ERROR:  load identification failed!" << endl;
		return LOAD_IDEN_CalErr;
	}
	le = fabs(tor6) / Me - l7;
	z = le;
	double mrz = z;
	double r = 0, theta = 0;
	double tor7_1, tor7_2;
	tor7_1 = loadTor.torLoad[2] - loadTor.torNoLoad[2];
	tor7_2 = loadTor.torLoad[3] - loadTor.torNoLoad[3];
	r = sqrt(pow(tor7_1, 2) + pow(tor7_2, 2)) / Me;
	double si = 0, cs = 0;
	if (r < 1e-6) {
		r = 0;
		theta = 0;
	}
	else {
		si = -tor7_2 / (Me * r);
		cs = tor7_1 / (Me * r);
		theta = atan2(si, cs);
	}
	double mrx = -r * sin(theta);
	double mry = r * cos(theta);
	double z5 = z + l7;
	double ixx6, icxx;
	double ixx_tor6;
	ixx_tor6 = loadTor.torLoad[4] - loadTor.torNoLoad[4];
	ixx6 = fabs(ixx_tor6 / acc);
	icxx = ixx6 - me * (z5 * z5 + mry * mry);
	double ixx = icxx + me * (z * z + mry * mry);
	double iyy6, icyy;
	double iyy_tor6;
	iyy_tor6 = loadTor.torLoad[5] - loadTor.torNoLoad[5];
	iyy6 = fabs(iyy_tor6 / acc);
	icyy = iyy6 - me * (z5 * z5 + mrx * mrx);
	double iyy = icyy + me * (z * z + mrx * mrx);
	double izz, iczz;
	double izz_tor7;
	izz_tor7 = loadTor.torLoad[6] - loadTor.torNoLoad[6];
	izz = fabs(izz_tor7 / acc);
	iczz = izz - me * (r * r);

	if (loadDyn == NULL) {
		cout << "ERROR: load identification failed!" << endl;
		return LOAD_IDEN_CalErr;
	}
	loadDyn->m = me;
	loadDyn->mr[0] = mrx * 1000;
	loadDyn->mr[1] = mry * 1000;
	loadDyn->mr[2] = mrz * 1000;
	loadDyn->inertia[0] = (ixx>0)? ixx:0.001;
    loadDyn->inertia[1] = (iyy > 0) ? iyy : 0.001;
    loadDyn->inertia[2] = (izz > 0) ? izz : 0.001;
	// printf(" m=%7.3fkg\n x=%7.3fmm\n y=%7.3fmm\n z=%7.3fmm\n ixx=%7.3f\n iyy=%7.3f\n izz=%7.3f\n", me, mrx, mry, mrz, ixx, iyy, izz);
	cout << "Load identification calculate successful!"<< endl;
	return LOAD_IDEN_NoErr;
}

static bool strSpliteCCS(const string& str, vector<vector<string>>& loadData)
{
	deque<string> deqItem;
	stringstream ss(str);
	string tem;
	const size_t dataCloNum = 9;
	int dataPos = 0;
	while (getline(ss, tem, ',')) {
		dataPos++;
		if (4 == dataPos ||
			5 == dataPos ||
			6 == dataPos ||
			7 == dataPos ||
			11 == dataPos ||
			12 == dataPos ||
			13 == dataPos ||
			14 == dataPos
			) {
			deqItem.push_back(tem);
		}
		if (15 == dataPos) {
			deqItem.push_front(tem);
			dataPos = 0;
		}
	}
	if (deqItem.size() < dataCloNum || loadData.size() < dataCloNum) {
		cout << "Data must be 9 columns" << endl;
		return false;
	}
	for (int i = 0; i < loadData.size(); i++) {
		loadData.at(i).push_back(deqItem.at(i));
	}
	return true;
}

LoadIdenErrCode OnCalLoadTor_7_MarvinCCS(LoadTor* tor, int isNoLoad, const char *userPath)
{
	if (tor == NULL) {
		printf("tor is null!\n");
		return LOAD_IDEN_CalErr;
	}
	string strUserPath(userPath);
	string strLDFileName("/LoadData.csv");
	string strNoLDFileName("/NoLoadData.csv");
	string tmpPath;
	if (1 == isNoLoad) {
		printf("Calculating no-load data...\n");
		tmpPath = strUserPath + strNoLDFileName;
	}
	else {
		tmpPath = strUserPath + strLDFileName;
	}
	ifstream ifLoadFile(tmpPath, ios::in);
	if (!ifLoadFile.is_open()) {
		cout << "ERROR: open file failed, pls check file! " << endl;
		return LOAD_IDEN_OpenSmpDateFieErr;
	}
	string strLoadDataLine;
	vector<vector<string>> vecLoadData(9);
	while (getline(ifLoadFile, strLoadDataLine)) {
		if (strSpliteCCS(strLoadDataLine, vecLoadData) == false) {
			cout << "ERROR: parse file failed, pls check file! "<< endl;
			return LOAD_IDEN_OpenSmpDateFieErr;
		}
	}
	ifLoadFile.close();
	string strIdenCfgName(strUserPath+"/CfgFile/LoadIdenCfg_Marvin_CCS.txt");
	ifstream ifsConFile(strIdenCfgName, ios::in);
	if (!ifsConFile.is_open()) {
		cout << "ERROR: open file failed, pls check :LoadIdenCfg_MarvinCCS.txt" << endl;
		return LOAD_IDEN_OpenCfgFileErr;
	}
	string strCfg;
	if (!getline(ifsConFile, strCfg)) {
		cout << "Error: load config failed, type-1, pls contact supplier for help." << endl;
		return LOAD_IDEN_OpenCfgFileErr;
	}
	ifsConFile.close();
	vector<string> vecCon;
	vecCon = strSplite(strCfg, ',');
	if (vecCon.size() != 17) {
		cout << "Error: load config failed, type-2, pls contact supplier for help." << endl;
		return LOAD_IDEN_OpenCfgFileErr;
	}
	const double torTag[7] = { 0, 1111, 2222, 3333, 4444, 5555, 6666 };
	double trajCondation[10] = { 1, -1, 1, 0, 5, 4, 95, 94, 20, -20 };
	double sampleCy = 1000;
	int dataTagPos = 0;
	vector<double> vecTor;
	double dtemp = 0;
	int i;
	int LoadDataNum = static_cast<int>(vecLoadData.at(0).size());
	int trajCalTimes = 0;
	double tor7[7] = { 0 };
	int dataFlagCount = 0;
	vecTor.clear();
	for (i = dataTagPos; i < LoadDataNum; i++) {
		if (torTag[1] == stod(vecLoadData.at(0).at(i))) {
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(1).at(i));
				if (dtemp<trajCondation[0] && dtemp>trajCondation[1]) {
					break;
				}
				i++;
			}
			trajCalTimes++;
			while (i < LoadDataNum) {
				dtemp = stod(vecLoadData.at(1).at(i));
				if (dtemp<trajCondation[0] && dtemp>trajCondation[1]) {
					vecTor.push_back(stod(vecLoadData.at(5).at(i)));
					i++;
				}
				else {
					dataTagPos = i;
					break;
				}

			}
		}
		if (2 == trajCalTimes) {
			trajCalTimes = 0;
			dataFlagCount++;
			break;
		}
	}
	dtemp = 0;
	for (int num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[0] = dtemp / vecTor.size();
	vecTor.clear();
	for (i = dataTagPos; i < LoadDataNum; i++) {
		if (torTag[2] == stod(vecLoadData.at(0).at(i))) {
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(3).at(i));
				if (dtemp<trajCondation[2] && dtemp>trajCondation[3]) {
					break;
				}
				i++;
			}
			trajCalTimes++;
			while (i < LoadDataNum) {
				dtemp = stod(vecLoadData.at(3).at(i));
				if (dtemp<trajCondation[2] && dtemp>trajCondation[3]) {
					vecTor.push_back(stod(vecLoadData.at(7).at(i)));
					i++;
				}
				else {
					dataTagPos = i;
					break;
				}
			}
		}
		if (2 == trajCalTimes) {
			trajCalTimes = 0;
			dataFlagCount++;
			break;
		}
	}
	dtemp = 0;
	for (int num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[1] = dtemp / vecTor.size();
	vecTor.clear();
	for (i = dataTagPos; i < LoadDataNum; i++) {
		if (torTag[3] == stod(vecLoadData.at(0).at(i))) {
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(2).at(i));
				if (dtemp<trajCondation[4] && dtemp>trajCondation[5]) {
					break;
				}
				i++;
			}
			trajCalTimes++;
			while (i < LoadDataNum) {
				dtemp = stod(vecLoadData.at(2).at(i));
				if (dtemp<trajCondation[4] && dtemp>trajCondation[5]) {
					vecTor.push_back(stod(vecLoadData.at(6).at(i)));
					i++;
				}
				else {
					dataTagPos = i;
					break;
				}
			}
		}
		if (2 == trajCalTimes) {
			trajCalTimes = 0;
			dataFlagCount++;
			break;
		}
	}
	dtemp = 0;
	for (int num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[2] = dtemp / vecTor.size();
	vecTor.clear();
	for (i = dataTagPos; i < LoadDataNum; i++) {
		if (torTag[3] == stod(vecLoadData.at(0).at(i))) {
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(2).at(i));
				if (dtemp<trajCondation[6] && dtemp>trajCondation[7]) {
					break;
				}
				i++;
			}
			trajCalTimes++;
			while (i < LoadDataNum) {
				dtemp = stod(vecLoadData.at(2).at(i));
				if (dtemp<trajCondation[6] && dtemp>trajCondation[7]) {
					vecTor.push_back(stod(vecLoadData.at(6).at(i)));
					i++;
				}
				else {
					dataTagPos = i;
					break;
				}
			}
		}
		if (2 == trajCalTimes) {
			trajCalTimes = 0;
			dataFlagCount++;
			break;
		}
	}
	dtemp = 0;
	for (int num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[3] = dtemp / vecTor.size();
	double dtemp2 = 0;
	bool inrange = false;
	vecTor.clear();
	inrange = false;
	for (i = dataTagPos; i < LoadDataNum - 1; i++) {
		if (torTag[4] == stod(vecLoadData.at(0).at(i))) {
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(2).at(i));
				dtemp2 = stod(vecLoadData.at(2).at(i + 1));
				if (((dtemp2 - dtemp) * sampleCy < trajCondation[8]) && (dtemp2 - dtemp) * sampleCy > trajCondation[9]) {
					break;
				}
				i++;
			}
			//
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(2).at(i));
				dtemp2 = stod(vecLoadData.at(2).at(i + 1));
				if (((dtemp2 - dtemp) * sampleCy < trajCondation[8]) && (dtemp2 - dtemp) * sampleCy > trajCondation[9]) {
					vecTor.push_back(stod(vecLoadData.at(6).at(i)));
					inrange = true;
				}
				else {
					dataTagPos = i;
					break;
				}
				i++;
			}
		}
		if (inrange) {
			dataTagPos = i;
			dataFlagCount++;
			break;
		}
	}
	dtemp = 0;
	for (int num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[4] = dtemp / vecTor.size();

	inrange = false;
	vecTor.clear();
	for (i = dataTagPos; i < LoadDataNum - 1; i++) {
		if (torTag[5] == stod(vecLoadData.at(0).at(i))) {
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(3).at(i));
				dtemp2 = stod(vecLoadData.at(3).at(i + 1));
				if (((dtemp2 - dtemp) * sampleCy < trajCondation[8]) && (dtemp2 - dtemp) * sampleCy > trajCondation[9]) {
					break;
				}
				i++;
			}
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(3).at(i));
				dtemp2 = stod(vecLoadData.at(3).at(i + 1));
				if (((dtemp2 - dtemp) * sampleCy < trajCondation[8]) && (dtemp2 - dtemp) * sampleCy > trajCondation[9]) {
					vecTor.push_back(stod(vecLoadData.at(7).at(i)));
					inrange = true;
				}
				else {
					dataTagPos = i;
					break;
				}
				i++;
			}
		}
		if (inrange) {
			dataTagPos = i;
			dataFlagCount++;
			break;
		}
	}
	dtemp = 0;
	for (int num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[5] = dtemp / vecTor.size();
	vecTor.clear();
	inrange = false;
	for (i = dataTagPos; i < LoadDataNum - 1; i++) {

		if (torTag[6] == stod(vecLoadData.at(0).at(i))) {
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(4).at(i));
				dtemp2 = stod(vecLoadData.at(4).at(i + 1));
				if (((dtemp2 - dtemp) * sampleCy < trajCondation[8]) && (dtemp2 - dtemp) * sampleCy > trajCondation[9]) {
					break;
				}
				i++;
			}
			while (i < LoadDataNum - 1) {
				dtemp = stod(vecLoadData.at(4).at(i));
				dtemp2 = stod(vecLoadData.at(4).at(i + 1));
				if (((dtemp2 - dtemp) * sampleCy < trajCondation[8]) && (dtemp2 - dtemp) * sampleCy > trajCondation[9]) {
					vecTor.push_back(stod(vecLoadData.at(8).at(i)));
					inrange = true;
				}
				else {
					dataTagPos = i;
					break;
				}
				i++;
			}
		}
		if (inrange) {
			dataTagPos = i;
			dataFlagCount++;
			break;
		}
	}
	dtemp = 0;

	if (dataFlagCount != 7) {
		cout << "ERROR: insufficient effective data, pls check LoadData.csv or NoLoadData.csv" << endl;
		return LOAD_IDEN_DataSmpErr;
	}

	for (int num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[6] = dtemp / vecTor.size();
	for (int i = 0; i < 7; i++) {
		tor->torNoLoad[i] = stod(vecCon.at(i + 10));
		tor->torLoad[i] = tor7[i];
	}
	if (1 == isNoLoad) {
		for (int i = 0; i < 7; i++) {
			tor->torNoLoad[i] = tor7[i];
			vecCon.at(i + 10) = to_string(tor7[i]);
			tor->torLoad[i] = 0;
		}
		ofstream ofsConFile(strIdenCfgName, ios::out);
		if (!ofsConFile.is_open()) {
			cout << "ERROR: open cfg file failed, pls check LoadIdenCfg" << endl;
			return LOAD_IDEN_DataSmpErr;
		}
		for (int i = 0; i < vecCon.size() - 1; i++) {
			if (!(ofsConFile << vecCon.at(i) << ",")) {
				cout << "ERROR: write cfg file failed, pls check LoadIdenCfg" << endl;
				return LOAD_IDEN_DataSmpErr;
			}
		}
		ofsConFile << vecCon.at(vecCon.size() - 1) << "\n";
		ofsConFile.close();
		cout << "Set no-load parameters finished" << endl;
	}
	return LOAD_IDEN_NoErr;
}

LoadIdenErrCode OnCalLoadDynamic_7_MarvinCCS(LoadDynamic* loadDyn, const char *userPath)
{
	cout << "Calculating load data..." << endl;
	LoadTor loadTor;
	memset(&loadTor, 0, sizeof(loadTor));
	LoadIdenErrCode retCode = LOAD_IDEN_NoErr;
	retCode = OnCalLoadTor_7_MarvinCCS(&loadTor, 0, userPath);
	if (retCode != LOAD_IDEN_NoErr) {
		cout << "ERROR: Calculated load identification parameters failed!"  << endl;
		return retCode;
	}


	// if (OnCalLoadTor_7_MarvinCCS(&loadTor, 0, userPath) == false) {
	// 	cout << "ERROR: Calculated load identification parameters failed!"  << endl;
	// 	return false;
	// }
	double g = 9.8, pi = 3.1415926;
	double acc = 800.0 / 180 * pi;
	double l4 = 314 / 1000.0;
	double l7 = 95 / 1000.0;
	double tor4, tor6;
	const double calErr = 0;
	tor4 = (loadTor.torLoad[0] - loadTor.torNoLoad[0]) + calErr;
	tor6 = (loadTor.torLoad[1] - loadTor.torNoLoad[1]);
	double Me, me, le, x;
	Me = (fabs(tor4) - fabs(tor6)) / l4;
	me = Me / g;
	if (me < 0.001 || me > 50 || isnan(me)) {
		cout << "ERROR: Calculated load identification parameters failed!" << endl;
		return LOAD_IDEN_CalErr;
	}
	le = fabs(tor6) / Me - l7;
	x = le;
	double mrx = x;
	double r = 0, theta = 0;
	double tor7_1, tor7_2;
	tor7_1 = loadTor.torLoad[2] - loadTor.torNoLoad[2];
	tor7_2 = loadTor.torLoad[3] - loadTor.torNoLoad[3];
	r = sqrt(pow(tor7_1, 2) + pow(tor7_2, 2)) / Me;
	double si = 0, cs = 0;
	if (r < 1e-6) {
		r = 0;
		theta = 0;
	}
	else {
		si = -tor7_2 / (Me * r);
		cs = tor7_1 / (Me * r);
		theta = atan2(si, cs);
	}
	double mrz = -r * sin(theta);
	double mry = -r * cos(theta);
	double x5 = x + l7;
	double ixx, icxx;
	double ixx_tor6;
	ixx_tor6 = loadTor.torLoad[4] - loadTor.torNoLoad[4];
	ixx = fabs(ixx_tor6 / acc);
	icxx = ixx - me * (r * r);
	double iyy6, icyy;
	double iyy_tor6;
	iyy_tor6 = loadTor.torLoad[5] - loadTor.torNoLoad[5];
	iyy6 = fabs(iyy_tor6 / acc);
	icyy = iyy6 - me * (x5 * x5 + mrz * mrz);
	double iyy = icyy + me * (x * x + mrz * mrz);
	double izz, iczz;
	double izz_tor7;
	izz_tor7 = loadTor.torLoad[6] - loadTor.torNoLoad[6];
	izz = fabs(izz_tor7 / acc);
	iczz = izz - me * (x5 * x5 + mry * mry);
	izz = iczz + me * (x * x + mry * mry);
	if (loadDyn == NULL) {
		cout << "ERROR: Calculated load identification parameters failed!" << endl;
		return LOAD_IDEN_CalErr;
	}
	mrx *= 1000;
	mry *= 1000;
	mrz *= 1000;
    double r6[3] = {mrx, mry, mrz};
    double transT6f[3][3] = {0, 0, 1, 0, -1, 0, 1, 0, 0};
    double rf[3] = { 0 };
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            rf[i] += transT6f[i][j] * r6[j];
        }
    }
    double ixxf = izz;
    double iyyf = iyy;
    double izzf = ixx;
    loadDyn->m = me;
	loadDyn->mr[0] = rf[0];
	loadDyn->mr[1] = rf[1];
	loadDyn->mr[2] = rf[2];
	loadDyn->inertia[0] = (ixxf > 0) ? ixxf : 0.001;
    loadDyn->inertia[1] = (iyyf > 0) ? iyyf : 0.001;
    loadDyn->inertia[2] = (izzf > 0) ? izzf : 0.001;
    printf(" m=%7.3fkg\n x=%7.3fmm\n y=%7.3fmm\n z=%7.3fmm\n ixx=%7.3f\n iyy=%7.3f\n izz=%7.3f\n", me, rf[0], rf[1], rf[2], ixxf, iyyf, izzf);
	cout << "Calculated load identification parameters successful" << endl;
	return LOAD_IDEN_NoErr;
}

LoadIdenErrCode OnCalLoadDyn(LoadDynamicPara *DynPara, FX_INT32 RobotType,  const FX_CHAR* UserPath)
{
	DynPara->m =    0;
	DynPara->r[0] = 0;
	DynPara->r[1] = 0;
	DynPara->r[2] = 0;
	DynPara->I[0] = 0;
	DynPara->I[1] = 0;
	DynPara->I[2] = 0;
	DynPara->I[3] = 0;
	DynPara->I[4] = 0;
	DynPara->I[5] = 0;
	LoadTor tor;
	LoadDynamic dy;
	LoadIdenErrCode retCode = LOAD_IDEN_NoErr;
	if (1 == RobotType) {
		retCode = OnCalLoadTor_7_MarvinCCS(&tor, 1, UserPath);
		if (retCode != LOAD_IDEN_NoErr) {
			return retCode;
		}
		retCode = OnCalLoadDynamic_7_MarvinCCS(&dy, UserPath);
		if (retCode != LOAD_IDEN_NoErr) {
			return retCode;
		}

		// if (OnCalLoadTor_7_MarvinCCS(&tor, 1, UserPath) == false) {
		// 	return FX_FALSE;
		// }
		// if (OnCalLoadDynamic_7_MarvinCCS(&dy, UserPath) == false) {
		// 	return FX_FALSE;
		// }
	}
	else if (2 == RobotType) {
		retCode = OnCalLoadTor_7_MarvinSRS(&tor, 1, UserPath);
		if (retCode != LOAD_IDEN_NoErr) {
			return retCode;
		}
		retCode = OnCalLoadDynamic_7_MarvinSRS(&dy, UserPath);
		if (retCode != LOAD_IDEN_NoErr) {
			return retCode;
		}

		// if (OnCalLoadTor_7_MarvinSRS(&tor, 1, UserPath) == false) {
		// 	return FX_FALSE;
		// }
		// if (OnCalLoadDynamic_7_MarvinSRS(&dy, UserPath) == false) {
		// 	return FX_FALSE;
		// }
	}
	else{

	}
	DynPara->m = dy.m;
	DynPara->r[0] = dy.mr[0];
	DynPara->r[1] = dy.mr[1];
	DynPara->r[2] = dy.mr[2];
	DynPara->I[0] = dy.inertia[0];
	DynPara->I[1] = dy.inertia[1];
	DynPara->I[2] = dy.inertia[2];
	return LOAD_IDEN_NoErr;
}
