// Computes the system response to a step in the motors to position (712,512,312).
// Reads 100 samples of IMUa and motor data, synchronised by motor cycle.

#include "stdafx.h"

#include <iostream>
#include <boost/asio.hpp>
#include <sstream>
#include <iomanip>
#include <time.h>
#include <string.h>
#include <stdio.h>


using namespace std;
using namespace boost::asio;
//using namespace chrono_literals;
/*-------------------------------------------------------*/
#include <cstdio>
#include <thread>
//#ifdef _WIN32
#include <LpmsSensorI.h>
#include <LpmsSensorManagerI.h>
//#endif
//#ifdef __GNUC__
//#include "lpsensor/LpmsSensorI.h"
//#include "lpsensor/LpmsSensorManagerI.h"
//#endif
/*---------------------------------------------------------*/

int main(int argc, char *argv[]) {

	time_t sec1;
	time_t sec2 = 0;

	ofstream outfile("dados.txt");
	std::vector<string> ms;
	std::vector<double> qw, qx, qy, qz, x, y, z;
	int memoria = 800;  //800 strings em memoria
	ms.reserve(memoria);
	qw.reserve(memoria);
	qx.reserve(memoria);
	qy.reserve(memoria);
	qz.reserve(memoria);
	x.reserve(memoria);
	y.reserve(memoria);
	z.reserve(memoria);

	printf("OLA_1");

	// Gets a LpmsSensorManager instance
	LpmsSensorManagerI* manager = LpmsSensorManagerFactory();
	printf("OLA_2\n\n");

	//
	// DEVICE_LPMS_B        LPMS-B (Bluetooth)
	// DEVICE_LPMS_U        LPMS-CU / LPMS-USBAL (USB)
	// DEVICE_LPMS_C        LPMS-CU / LPMS-CANAL(CAN bus)
	// DEVICE_LPMS_BLE      LPMS-BLE (Bluetooth low energy)
	// DEVICE_LPMS_RS232    LPMS-UARTAL (RS-232)
	// DEVICE_LPMS_B2       LPMS-B2
	// DEVICE_LPMS_U2       LPMS-CU2/URS2/UTTL2/USBAL2 (USB)
	// DEVICE_LPMS_C2       LPMS-CU2/CANAL2 (CAN)

	// Connects to LPMS-B2 sensor with address 00:11:22:33:44:55
	//LpmsSensorI* lpms = manager->addSensor(DEVICE_LPMS_B2, "00:11:22:33:44:55");
	LpmsSensorI* lpms;

	printf("OLA_3\n\n");

	lpms = manager->addSensor(DEVICE_LPMS_U, "A1019SCW");

	printf("OLA_4\n\n\n");

	io_service io;
	serial_port sp(io);
	boost::system::error_code ec;
	sp.open("COM4", ec);
	if (ec) { cout << "Error_1"; return -1; }

	sp.set_option(serial_port_base::baud_rate(9600), ec);
	if (ec) { cout << "Error_2"; return -1; }

	printf("OLA");
	ImuData d;

	while (1)
	{
		Sleep(1000);
		if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED && lpms->hasImuData())
			break;
	}

	Sleep(2000);

	/*-------------------------------------------------------------------- */

	int pos1 = 712;
	int pos2 = 512;
	int pos3 = 312;
	//const int step = 80;

	//for (pos1; pos1 < 1000; pos1 = pos1 + step)
	//{
	//	for (pos2 = 300; pos2 < 800; pos2 = pos2 + step)
	//	{
	//		for (pos3 = 160; pos3 < 1000; pos3 = pos3 + step)
	//		{
	cout << "------------------------COMECA--------------------------\n";

	stringstream s1;
	s1 << setw(4) << setfill('0') << pos1; //enche para a forma 0050
	string sis1 = s1.str();
	cout << "string_1 : " << sis1;

	stringstream s2;
	s2 << setw(4) << setfill('0') << pos2; //enche para a forma 0050
	string sis2 = s2.str();
	cout << " string_2 : " << sis2;

	//pos2 += step;
	stringstream s3;
	s3 << setw(4) << setfill('0') << pos3; //enche para a forma 0050
	string sis3 = s3.str();
	cout << " string_3 : " << sis3;

	putchar('\n');

	string total;
	total = sis1 + sis2 + sis3;
	cout << "total : " << total;
	putchar('\n');

	write(sp, boost::asio::buffer(total)); //escreve para o serialUSB

	sec1 = time(NULL);
	//int i = 0;
	d = lpms->getCurrentData();
	float aux = d.r[2];
	qw.push_back(d.q[0]);
	qx.push_back(d.q[1]);
	qy.push_back(d.q[2]);
	qz.push_back(d.q[3]);
	x.push_back(d.r[0]);
	//cout <<"aqui esta:"<< x[i]<<"\n"<<endl;
	//i++;
	y.push_back(d.r[1]);
	z.push_back(d.r[2]);

	int i = 0;
	while (i < 100) {


		//float cloc = clock();

		boost::asio::streambuf leu;
		read_until(sp, leu, '\n', ec);// lê do serialUSB
		if (ec) { cout << ec.message(); return -1; }
		//cout << &leu;


		// Checks, if sensor is connected
		//if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED && lpms->hasImuData())
		//{
		//Sleep(2000);
		  // Reads quaternion data
		d = lpms->getCurrentData();

		//cout << "\n tempo : " << (clock() - cloc) << endl;

		// Shows data
			//printf("Timestamp=%f, qW=%f, qX=%f, qY=%f, qZ=%f, x=%f, y=%f, z=%f\n",
			//d.timeStamp, d.q[0], d.q[1], d.q[2], d.q[3],
			//d.r[0], d.r[1], d.r[2]);
	//}

		std::istream is(&leu);
		std::string escreve;
		std::getline(is, escreve);
		cout << escreve<<endl;

		// Guardamos em memória os dados
		ms.push_back(escreve);
		qw.push_back(d.q[0]);
		qx.push_back(d.q[1]);
		qy.push_back(d.q[2]);
		qz.push_back(d.q[3]);
		x.push_back(d.r[0]);
		//cout <<"aqui esta:"<< x[i]<<"\n"<<endl;
		//i++;
		y.push_back(d.r[1]);
		z.push_back(d.r[2]);

		//cout << clock() - cloc<<endl;


		//sec2 = time(NULL);
		i = i + 1;

		//cout << i<<endl;

		//do something that takes a few milliseconds
	}

	//}
//}
//}
	cout << "------------------------END--------------------------\n";

	putchar('\n');

	Sleep(9000);
	//}
	//}
	//}

	int amostras = ms.size();

	for (int linha = 0; linha < amostras; linha++)
	{
		outfile << "" << ms[linha] << " " << qw[linha] << " " << qx[linha] << " " << qy[linha] << " " << qz[linha] << " " << x[linha] << " " << y[linha] << " " << z[linha] << endl;
		//outfile << "" << ms[linha] << " " << x[linha] << " " << y[linha] << " " << z[linha] << endl;
	}

	outfile.close();

	/*ifstream infile("dados.txt");
	std::vector<int> M1, M2, M3;
	std::vector<double> W0, X1, Y2, Z3, X, Y, Z;
	string STRING;
	string lixo;
	int m1, m2, m3;
	double w0, x1, y2, z3;
	double x, y, z;
	int i = 0;
	while (!infile.eof()) // To get you all the lines.
	{
	getline(infile, STRING); // Saves the line in STRING.

	std::stringstream convertor(STRING);

	convertor >> lixo >> lixo >> lixo >> m1 >> lixo >> lixo >> m2 >> lixo >> lixo >> m3 >> lixo >> lixo >> lixo >> w0 >> lixo >> lixo >> lixo >> x1 >> lixo >> lixo >> lixo >> y2 >> lixo >> lixo >> lixo >> z3 >> lixo >> lixo >> lixo >> x >> lixo >> lixo >> lixo >> y >> lixo >> lixo >> lixo >> z;
	M1.push_back(m1);
	M2.push_back(m2);
	M3.push_back(m3);
	W0.push_back(w0);
	X1.push_back(x1);
	Y2.push_back(y2);
	Z3.push_back(z3);
	X.push_back(x);
	Y.push_back(y);
	Z.push_back(z);
	cout << M1[i] << " "<<M2[i] <<" "<< M3[i] << " "<<W0[i] << " " << X1[i] << " " << Y2[i] << " " << Z3[i] << " " << X[i] << " " << Y[i] << " " << Z[i] << "\n";
	//cout << M1[i] << " " << M2[i] << " " << M3[i] << "\n";
	i++;
	//Sleep(1000);
	}
	infile.close(); */
}
