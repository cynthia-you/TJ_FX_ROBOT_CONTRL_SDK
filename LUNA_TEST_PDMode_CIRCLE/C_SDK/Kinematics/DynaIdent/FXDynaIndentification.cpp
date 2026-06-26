#include "FXDynaIndentification.h"
#include "../KineCommon/FXLog.h"
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

/* C++98 compatibility helpers — std::stod / std::to_string are C++11.
   Only defined when compiling in pre-C++11 mode; on modern compilers
   the standard library provides them, and redefining causes ambiguity. */
#if __cplusplus < 201103L
static double stod(const string& s) { return atof(s.c_str()); }
static string to_string(double d) {
    ostringstream oss;
    oss << d;
    return oss.str();
}
#endif

typedef struct {
	FX_DOUBLE torLoad[7];
	FX_DOUBLE torNoLoad[7];
}LoadTor;

typedef struct {
	FX_DOUBLE m;
	FX_DOUBLE mr[3];
	FX_DOUBLE inertia[6];
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

static bool strSpliteSRS(const string& str, vector<vector<string> >& loadData)
{
	deque<string> deqItem;
	stringstream ss(str);
	string tem;
	const size_t dataCloNum = 7;
	FX_INT32 dataPos = 0;
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
		FX_LOG_ERRO("strSpliteSRS: data must be 7  columns");
		return FX_FALSE;
	}
	for (size_t i = 0; i < loadData.size(); i++) {
		loadData.at(i).push_back(deqItem.at(i));
	}
	return FX_TRUE;
}

LoadIdenErrCode OnCalLoadTor_7_MarvinSRS(LoadTor* tor, FX_INT32 isNoLoad, const char *userPath)
{
	if (tor == NULL) {
		FX_LOG_ERRO("OnCalLoadTor_7_MarvinSRS: tor is null!");
		return LOAD_IDEN_CalErr;
	}
	string strUserPath(userPath);
	string strLDFileName("/LoadData.csv");
	string strNoLDFileName("/NoLoadData.csv");
	string tmpPath;
	if (1 == isNoLoad) {
		FX_LOG_INFO("Calculating no-load data...");
		tmpPath = strUserPath + strNoLDFileName;
	}
	else {
		tmpPath = strUserPath + strLDFileName;
	}
	ifstream ifLoadFile(tmpPath.c_str(), ios::in);
	if (!ifLoadFile.is_open()) {
		FX_LOG_ERRO("OnCalLoadTor_7_MarvinSRS: file load failed, pls check file!");
		return LOAD_IDEN_OpenSmpDateFieErr;
	}
	string strLoadDataLine;
	vector<vector<string> > vecLoadData(7);
	while (getline(ifLoadFile, strLoadDataLine)) {
		if (strSpliteSRS(strLoadDataLine, vecLoadData) == FX_FALSE) {
			FX_LOG_ERRO("OnCalLoadTor_7_MarvinSRS: parse file failed, pls check file!");
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

	ifstream ifsConFile(strIdenCfgName.c_str(), ios::in);
	if (!ifsConFile.is_open()) {
		FX_LOG_ERRO("OnCalLoadTor_7_MarvinSRS: load cfg file failed, pls check file<LoadIdenCfg_Marvin_SRS.txt>");
		return LOAD_IDEN_OpenCfgFileErr;
	}
	string strCfg;
	if (!getline(ifsConFile, strCfg)) {
		FX_LOG_ERRO("OnCalLoadTor_7_MarvinSRS: load config failed, type-1, pls contact supplier for help.");
		return LOAD_IDEN_OpenCfgFileErr;
	}
	ifsConFile.close();
	vector<string> vecCon;
	vecCon = strSplite(strCfg, ',');
	if (vecCon.size() != 17) {
		FX_LOG_ERRO("OnCalLoadTor_7_MarvinSRS: load config failed, type-2, pls contact supplier for help.");
		return LOAD_IDEN_OpenCfgFileErr;
	}
	const FX_DOUBLE torTag[7] = { 0, 1111, 2222, 3333, 4444, 5555, 6666 };
    FX_DOUBLE trajCondation[10] = { 1, 0, 1, 0, 5, 4, 95, 94, 20, -20 };
	FX_DOUBLE sampleCy = 1000;

	FX_INT32 dataTagPos = 0;
	vector<FX_DOUBLE> vecTor;
	FX_DOUBLE dtemp = 0;
	FX_INT32 i;
	FX_INT32 LoadDataNum = static_cast<FX_INT32>(vecLoadData.at(DataTag).size());
	FX_INT32 trajCalTimes = 0;
	FX_DOUBLE tor7[7] = { 0 };
	FX_INT32 dataFlagCount = 0;
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
	for (size_t num = 0; num < vecTor.size(); num++) {
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
	for (size_t num = 0; num < vecTor.size(); num++) {
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
	for (size_t num = 0; num < vecTor.size(); num++) {
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
	for (size_t num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[3] = dtemp / vecTor.size();
	FX_DOUBLE dtemp2 = 0;
	bool inrange = FX_FALSE;
	vecTor.clear();
	inrange = FX_FALSE;
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
					inrange = FX_TRUE;
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
	for (size_t num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[6] = dtemp / vecTor.size();
	vecTor.clear();
	inrange = FX_FALSE;
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
					inrange = FX_TRUE;
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
	for (size_t num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[5] = dtemp / vecTor.size();
	inrange = FX_FALSE;
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
					inrange = FX_TRUE;
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
		FX_LOG_ERRO("OnCalLoadTor_7_MarvinSRS: insufficient effective data, pls check LoadData.csv or NoLoadData.csv");
		return LOAD_IDEN_DataSmpErr;
	}
	for (size_t num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[4] = dtemp / vecTor.size();
	for (FX_INT32 i = 0; i < 7; i++) {
		tor->torNoLoad[i] = stod(vecCon.at(i + 10));
		tor->torLoad[i] = tor7[i];
	}
	if (1 == isNoLoad) {
		for (FX_INT32 i = 0; i < 7; i++) {
			tor->torNoLoad[i] = tor7[i];
			vecCon.at(i + 10) = to_string(tor7[i]);
			tor->torLoad[i] = 0;
		}
		ofstream ofsConFile(strIdenCfgName.c_str());
		if (!ofsConFile.is_open()) {
			FX_LOG_ERRO("OnCalLoadTor_7_MarvinSRS: pls check ./LoadData/CfgFileLoadIdenCfg");
			return LOAD_IDEN_OpenCfgFileErr;
		}
		for (size_t i = 0; i + 1 < vecCon.size(); i++) {
			if (!(ofsConFile << vecCon.at(i) << ",")) {
				FX_LOG_ERRO("OnCalLoadTor_7_MarvinSRS: pls check ./LoadData/CfgFileLoadIdenCfg");
				return LOAD_IDEN_OpenCfgFileErr;
			}
		}
		ofsConFile << vecCon.at(vecCon.size() - 1) << "\n";
		ofsConFile.close();
		FX_LOG_INFO("Set no-load parameters finished");
	}
	return LOAD_IDEN_NoErr;
}

LoadIdenErrCode OnCalLoadDynamic_7_MarvinSRS(LoadDynamic* loadDyn, const char *userPath)
{
	FX_LOG_INFO("Calculating load data...");
	LoadTor loadTor;
	memset(&loadTor, 0, sizeof(loadTor));

	LoadIdenErrCode retCode = LOAD_IDEN_NoErr;
	retCode = OnCalLoadTor_7_MarvinSRS(&loadTor, 0, userPath);
	if (retCode != LOAD_IDEN_NoErr) {
		return retCode;
	}

	FX_DOUBLE g = 9.8, pi = 3.1415926;
	FX_DOUBLE acc = 1000.0 / 180 * pi;
	FX_DOUBLE l4 = 280.0 / 1000.0;
	FX_DOUBLE l7 = 160.0 / 1000.0;
	FX_DOUBLE tor4, tor6;
	const FX_DOUBLE calErr = 0;
	tor4 = (loadTor.torLoad[0] - loadTor.torNoLoad[0]) + calErr;
	tor6 = (loadTor.torLoad[1] - loadTor.torNoLoad[1]);
	FX_DOUBLE Me, me, le, z;
	Me = (fabs(tor4) - fabs(tor6)) / l4;
	me = Me / g;
	if (me < 0.001 || me > 50 || isnan(me)) {
		FX_LOG_ERRO("OnCalLoadDynamic_7_MarvinSRS: load identification failed!");
		return LOAD_IDEN_CalErr;
	}
	le = fabs(tor6) / Me - l7;
	z = le;
	FX_DOUBLE mrz = z;
	FX_DOUBLE r = 0, theta = 0;
	FX_DOUBLE tor7_1, tor7_2;
	tor7_1 = loadTor.torLoad[2] - loadTor.torNoLoad[2];
	tor7_2 = loadTor.torLoad[3] - loadTor.torNoLoad[3];
	r = sqrt(pow(tor7_1, 2) + pow(tor7_2, 2)) / Me;
	FX_DOUBLE si = 0, cs = 0;
	if (r < 1e-6) {
		r = 0;
		theta = 0;
	}
	else {
		si = -tor7_2 / (Me * r);
		cs = tor7_1 / (Me * r);
		theta = atan2(si, cs);
	}
	FX_DOUBLE mrx = -r * sin(theta);
	FX_DOUBLE mry = r * cos(theta);
	FX_DOUBLE z5 = z + l7;
	FX_DOUBLE ixx6, icxx;
	FX_DOUBLE ixx_tor6;
	ixx_tor6 = loadTor.torLoad[4] - loadTor.torNoLoad[4];
	ixx6 = fabs(ixx_tor6 / acc);
	icxx = ixx6 - me * (z5 * z5 + mry * mry);
	FX_DOUBLE ixx = icxx + me * (z * z + mry * mry);
	FX_DOUBLE iyy6, icyy;
	FX_DOUBLE iyy_tor6;
	iyy_tor6 = loadTor.torLoad[5] - loadTor.torNoLoad[5];
	iyy6 = fabs(iyy_tor6 / acc);
	icyy = iyy6 - me * (z5 * z5 + mrx * mrx);
	FX_DOUBLE iyy = icyy + me * (z * z + mrx * mrx);
	FX_DOUBLE izz;
	FX_DOUBLE izz_tor7;
	izz_tor7 = loadTor.torLoad[6] - loadTor.torNoLoad[6];
	izz = fabs(izz_tor7 / acc);
	if (loadDyn == NULL) {
		FX_LOG_ERRO("OnCalLoadDynamic_7_MarvinSRS: load identification failed!");
		return LOAD_IDEN_CalErr;
	}
	loadDyn->m = me;
	loadDyn->mr[0] = mrx * 1000;
	loadDyn->mr[1] = mry * 1000;
	loadDyn->mr[2] = mrz * 1000;
	loadDyn->inertia[0] = (ixx>0)? ixx:0.001;
    loadDyn->inertia[1] = (iyy > 0) ? iyy : 0.001;
    loadDyn->inertia[2] = (izz > 0) ? izz : 0.001;
	FX_LOG_INFO("Load identification calculate successful!");
	return LOAD_IDEN_NoErr;
}

static bool strSpliteCCS(const string& str, vector<vector<string> >& loadData)
{
	deque<string> deqItem;
	stringstream ss(str);
	string tem;
	const size_t dataCloNum = 9;
	FX_INT32 dataPos = 0;
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
		FX_LOG_ERRO("strSpliteCCS: Data must be 9 columns");
		return FX_FALSE;
	}
	for (size_t i = 0; i < loadData.size(); i++) {
		loadData.at(i).push_back(deqItem.at(i));
	}
	return FX_TRUE;
}

LoadIdenErrCode OnCalLoadTor_7_MarvinCCS(LoadTor* tor, FX_INT32 isNoLoad, const char *userPath)
{
	if (tor == NULL) {
		FX_LOG_ERRO("OnCalLoadTor_7_MarvinCCS: tor is null!");
		return LOAD_IDEN_CalErr;
	}
	string strUserPath(userPath);
	string strLDFileName("/LoadData.csv");
	string strNoLDFileName("/NoLoadData.csv");
	string tmpPath;
	if (1 == isNoLoad) {
		FX_LOG_INFO("Calculating no-load data...");
		tmpPath = strUserPath + strNoLDFileName;
	}
	else {
		tmpPath = strUserPath + strLDFileName;
	}
	ifstream ifLoadFile(tmpPath.c_str(), ios::in);
	if (!ifLoadFile.is_open()) {
		FX_LOG_ERRO("OnCalLoadTor_7_MarvinCCS: open file failed, pls check file! ");
		return LOAD_IDEN_OpenSmpDateFieErr;
	}
	string strLoadDataLine;
	vector<vector<string> > vecLoadData(9);
	while (getline(ifLoadFile, strLoadDataLine)) {
		if (strSpliteCCS(strLoadDataLine, vecLoadData) == FX_FALSE) {
			FX_LOG_ERRO("OnCalLoadTor_7_MarvinCCS: parse file failed, pls check file! ");
			return LOAD_IDEN_OpenSmpDateFieErr;
		}
	}
	ifLoadFile.close();
	string strIdenCfgName(strUserPath+"/CfgFile/LoadIdenCfg_Marvin_CCS.txt");
	ifstream ifsConFile(strIdenCfgName.c_str(), ios::in);
	if (!ifsConFile.is_open()) {
		FX_LOG_ERRO("OnCalLoadTor_7_MarvinCCS: open file failed, pls check :LoadIdenCfg_MarvinCCS.txt");
		return LOAD_IDEN_OpenCfgFileErr;
	}
	string strCfg;
	if (!getline(ifsConFile, strCfg)) {
		FX_LOG_ERRO("OnCalLoadTor_7_MarvinCCS: load config failed, type-1, pls contact supplier for help.");
		return LOAD_IDEN_OpenCfgFileErr;
	}
	ifsConFile.close();
	vector<string> vecCon;
	vecCon = strSplite(strCfg, ',');
	if (vecCon.size() != 17) {
		FX_LOG_ERRO("OnCalLoadTor_7_MarvinCCS: load config failed, type-2, pls contact supplier for help.");
		return LOAD_IDEN_OpenCfgFileErr;
	}
	const FX_DOUBLE torTag[7] = { 0, 1111, 2222, 3333, 4444, 5555, 6666 };
	FX_DOUBLE trajCondation[10] = { 1, -1, 1, 0, 5, 4, 95, 94, 20, -20 };
	FX_DOUBLE sampleCy = 1000;
	FX_INT32 dataTagPos = 0;
	vector<FX_DOUBLE> vecTor;
	FX_DOUBLE dtemp = 0;
	FX_INT32 i;
	FX_INT32 LoadDataNum = static_cast<FX_INT32>(vecLoadData.at(0).size());
	FX_INT32 trajCalTimes = 0;
	FX_DOUBLE tor7[7] = { 0 };
	FX_INT32 dataFlagCount = 0;
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
	for (size_t num = 0; num < vecTor.size(); num++) {
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
	for (size_t num = 0; num < vecTor.size(); num++) {
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
	for (size_t num = 0; num < vecTor.size(); num++) {
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
	for (size_t num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[3] = dtemp / vecTor.size();
	FX_DOUBLE dtemp2 = 0;
	bool inrange = FX_FALSE;
	vecTor.clear();
	inrange = FX_FALSE;
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
					inrange = FX_TRUE;
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
	for (size_t num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[4] = dtemp / vecTor.size();

	inrange = FX_FALSE;
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
					inrange = FX_TRUE;
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
	for (size_t num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[5] = dtemp / vecTor.size();
	vecTor.clear();
	inrange = FX_FALSE;
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
					inrange = FX_TRUE;
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
		FX_LOG_ERRO("OnCalLoadTor_7_MarvinCCS: insufficient effective data, pls check LoadData.csv or NoLoadData.csv");
		return LOAD_IDEN_DataSmpErr;
	}

	for (size_t num = 0; num < vecTor.size(); num++) {
		dtemp += vecTor.at(num);
	}
	tor7[6] = dtemp / vecTor.size();
	for (FX_INT32 i = 0; i < 7; i++) {
		tor->torNoLoad[i] = stod(vecCon.at(i + 10));
		tor->torLoad[i] = tor7[i];
	}
	if (1 == isNoLoad) {
		for (FX_INT32 i = 0; i < 7; i++) {
			tor->torNoLoad[i] = tor7[i];
			vecCon.at(i + 10) = to_string(tor7[i]);
			tor->torLoad[i] = 0;
		}
		ofstream ofsConFile(strIdenCfgName.c_str(), ios::out);
		if (!ofsConFile.is_open()) {
			FX_LOG_ERRO("OnCalLoadTor_7_MarvinCCS: open cfg file failed, pls check LoadIdenCfg");
			return LOAD_IDEN_DataSmpErr;
		}
		for (size_t i = 0; i + 1 < vecCon.size(); i++) {
			if (!(ofsConFile << vecCon.at(i) << ",")) {
				FX_LOG_ERRO("OnCalLoadTor_7_MarvinCCS: write cfg file failed, pls check LoadIdenCfg");
				return LOAD_IDEN_DataSmpErr;
			}
		}
		ofsConFile << vecCon.at(vecCon.size() - 1) << "\n";
		ofsConFile.close();
		FX_LOG_INFO("Set no-load parameters finished");
	}
	return LOAD_IDEN_NoErr;
}

LoadIdenErrCode OnCalLoadDynamic_7_MarvinCCS(LoadDynamic* loadDyn, const char *userPath)
{
	FX_LOG_INFO("Calculating load data...");
	LoadTor loadTor;
	memset(&loadTor, 0, sizeof(loadTor));
	LoadIdenErrCode retCode = LOAD_IDEN_NoErr;
	retCode = OnCalLoadTor_7_MarvinCCS(&loadTor, 0, userPath);
	if (retCode != LOAD_IDEN_NoErr) {
		FX_LOG_ERRO("OnCalLoadDynamic_7_MarvinCCS: Calculated load identification parameters failed!");
		return retCode;
	}

	FX_DOUBLE g = 9.8, pi = 3.1415926;
	FX_DOUBLE acc = 800.0 / 180 * pi;
	FX_DOUBLE l4 = 314 / 1000.0;
	FX_DOUBLE l7 = 95 / 1000.0;
	FX_DOUBLE tor4, tor6;
	const FX_DOUBLE calErr = 0;
	tor4 = (loadTor.torLoad[0] - loadTor.torNoLoad[0]) + calErr;
	tor6 = (loadTor.torLoad[1] - loadTor.torNoLoad[1]);
	FX_DOUBLE Me, me, le, x;
	Me = (fabs(tor4) - fabs(tor6)) / l4;
	me = Me / g;
	if (me < 0.001 || me > 50 || isnan(me)) {
		FX_LOG_ERRO("OnCalLoadDynamic_7_MarvinCCS: Calculated load identification parameters failed!");
		return LOAD_IDEN_CalErr;
	}
	le = fabs(tor6) / Me - l7;
	x = le;
	FX_DOUBLE mrx = x;
	FX_DOUBLE r = 0, theta = 0;
	FX_DOUBLE tor7_1, tor7_2;
	tor7_1 = loadTor.torLoad[2] - loadTor.torNoLoad[2];
	tor7_2 = loadTor.torLoad[3] - loadTor.torNoLoad[3];
	r = sqrt(pow(tor7_1, 2) + pow(tor7_2, 2)) / Me;
	FX_DOUBLE si = 0, cs = 0;
	if (r < 1e-6) {
		r = 0;
		theta = 0;
	}
	else {
		si = -tor7_2 / (Me * r);
		cs = tor7_1 / (Me * r);
		theta = atan2(si, cs);
	}
	FX_DOUBLE mrz = -r * sin(theta);
	FX_DOUBLE mry = -r * cos(theta);
	FX_DOUBLE x5 = x + l7;
	FX_DOUBLE ixx;
	FX_DOUBLE ixx_tor6;
	ixx_tor6 = loadTor.torLoad[4] - loadTor.torNoLoad[4];
	ixx = fabs(ixx_tor6 / acc);
	FX_DOUBLE iyy6, icyy;
	FX_DOUBLE iyy_tor6;
	iyy_tor6 = loadTor.torLoad[5] - loadTor.torNoLoad[5];
	iyy6 = fabs(iyy_tor6 / acc);
	icyy = iyy6 - me * (x5 * x5 + mrz * mrz);
	FX_DOUBLE iyy = icyy + me * (x * x + mrz * mrz);
	FX_DOUBLE izz, iczz;
	FX_DOUBLE izz_tor7;
	izz_tor7 = loadTor.torLoad[6] - loadTor.torNoLoad[6];
	izz = fabs(izz_tor7 / acc);
	iczz = izz - me * (x5 * x5 + mry * mry);
	izz = iczz + me * (x * x + mry * mry);
	if (loadDyn == NULL) {
		FX_LOG_ERRO("OnCalLoadDynamic_7_MarvinCCS: Calculated load identification parameters failed!");
		return LOAD_IDEN_CalErr;
	}
	mrx *= 1000;
	mry *= 1000;
	mrz *= 1000;
    FX_DOUBLE r6[3] = {mrx, mry, mrz};
    FX_DOUBLE transT6f[3][3] = {0, 0, 1, 0, -1, 0, 1, 0, 0};
    FX_DOUBLE rf[3] = { 0 };
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            rf[i] += transT6f[i][j] * r6[j];
        }
    }
    FX_DOUBLE ixxf = izz;
    FX_DOUBLE iyyf = iyy;
    FX_DOUBLE izzf = ixx;
    loadDyn->m = me;
	loadDyn->mr[0] = rf[0];
	loadDyn->mr[1] = rf[1];
	loadDyn->mr[2] = rf[2];
	loadDyn->inertia[0] = (ixxf > 0) ? ixxf : 0.001;
    loadDyn->inertia[1] = (iyyf > 0) ? iyyf : 0.001;
    loadDyn->inertia[2] = (izzf > 0) ? izzf : 0.001;

    FX_LOG_DEBG("<OnCalLoadDynamic_7_MarvinCCS> m=%7.3fkg x=%7.3fmm y=%7.3fmm z=%7.3fmm ixx=%7.3f iyy=%7.3f izz=%7.3f", me, rf[0], rf[1], rf[2], ixxf, iyyf, izzf);
	FX_LOG_INFO("Calculated load identification parameters successful");

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

		// if (OnCalLoadTor_7_MarvinCCS(&tor, 1, UserPath) == FX_FALSE) {
		// 	return FX_FALSE;
		// }
		// if (OnCalLoadDynamic_7_MarvinCCS(&dy, UserPath) == FX_FALSE) {
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

		// if (OnCalLoadTor_7_MarvinSRS(&tor, 1, UserPath) == FX_FALSE) {
		// 	return FX_FALSE;
		// }
		// if (OnCalLoadDynamic_7_MarvinSRS(&dy, UserPath) == FX_FALSE) {
		// 	return FX_FALSE;
		// }
	}
	else{
		return LOAD_INDEN_TypeErr;
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
