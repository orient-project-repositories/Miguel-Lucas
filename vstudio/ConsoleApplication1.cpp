// test
// Hi Alex, I added another line...
//
// Makes four repetitions of a cycle between motor setpoints:
// (410,682,703)
// (704,693,459)
// (709,336,367)
// (413,308,655)
// Reads motor and IMU data at motor control rate and saves to file

#include "stdafx.h"
#include "stdafx.h"
#include <iostream>
#include <boost/asio.hpp>
#include <sstream>
#include <iomanip>
#include <time.h>
#include <string.h>
#include <fstream>


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

	/*-------------------------------------------------------------*/
	io_service io;
	serial_port sp(io);
	boost::system::error_code ec;
	sp.open("COM4", ec);
	if (ec) { cout << "Error_1"; return -1; }

	sp.set_option(serial_port_base::baud_rate(9600), ec);
	if (ec) { cout << "Error_2"; return -1; }
	/*-----------------------------------------------------------*/
	printf("OLA");
	ImuData d;
	ImuData auxxx;

	printf("OLA_1");

	// Gets a LpmsSensorManager instance
	LpmsSensorManagerI* manager = LpmsSensorManagerFactory();
	printf("OLA_2\n\n");

	LpmsSensorI* lpms;

	printf("OLA_3\n\n");

	lpms = manager->addSensor(DEVICE_LPMS_U, "A1019SCW");

	printf("OLA_4\n\n\n");

	while (1)
	{
		Sleep(2000);
		if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED && lpms->hasImuData())
			break;
	}
	/*--------------------------------------------------------------------*/

	int pos1=612;
	int pos2=562;
	int pos3=412;
	const int step = 50;

	int ii, jj, kk;

	auxxx = lpms->getCurrentData();
	float aux = auxxx.r[2];

	for (ii = 1; ii <5; ii = ii +1)
	{
		pos1 = 410;
		pos2 = 682;
		pos3 = 703;

		for (jj = 1; jj <5; jj = jj+1)
		{
		
				cout << "------------------------COMECA--------------------------\n";
				stringstream s1;
				s1 << setw(4) << setfill('0') << pos1; //enche para a forma 0050
				string sis1 = s1.str();
				//cout << "string_1 : " << sis1;

				stringstream s2;
				s2 << setw(4) << setfill('0') << pos2; //enche para a forma 0050
				string sis2 = s2.str();
				//cout << " string_2 : " << sis2;

				stringstream s3;
				s3 << setw(4) << setfill('0') << pos3; //enche para a forma 0050
				string sis3 = s3.str();
				//cout << " string_3 : " << sis3;

				//putchar('\n');

				string total;
				total = sis1 + sis2 + sis3;
				cout << "total : " << total;
				putchar('\n');
				//Sleep(2000);

				write(sp, boost::asio::buffer(total)); //escreve para o serialUSB

				int i = 0;
				while (i < 30) {
					boost::asio::streambuf leu;
					read_until(sp, leu, '\n', ec);// lê do serialUSB
					if (ec) { cout << ec.message(); return -1; }

					/*----------------------------------------------------*/
					// Reads quaternion data
					d = lpms->getCurrentData();

					float azim = (d.r[2] - aux);

					if (d.r[2] > 100 && aux < -100)
						azim = -(180 - d.r[2] + aux + 180);

					if (d.r[2] < -100 && aux > 100)
						azim = (180 - d.r[2] + aux + 180);

					//cout << "d: " << d.r[2] << "\n" << "aux: " << aux << "\n" << "azim: " << azim << endl;


					// Shows data
					//printf("Timestamp=%f, qW=%f, qX=%f, qY=%f, qZ=%f, x=%f, y=%f, z=%f\n",
					//	d.timeStamp, d.q[0], d.q[1], d.q[2], d.q[3],
					//	d.r[0], d.r[1], (azim));

					std::istream is(&leu);
					std::string final;
					std::getline(is, final);

					//cout << "final : " << final << "\n";

					ms.push_back(final);
					qw.push_back(d.q[0]);
					qx.push_back(d.q[1]);
					qy.push_back(d.q[2]);
					qz.push_back(d.q[3]);
					x.push_back(d.r[0]);
					y.push_back(d.r[1]);
					z.push_back(azim);

					i = i + 1;
				}

				/*--------------------------------------------------------*/

				cout << "------------------------END--------------------------\n";


				putchar('\n');

				if (pos1 == 410 && pos2 == 682)
				{
					pos1 = 704; pos2 = 693; pos3 = 459;
				}
				else if (pos2 == 693 && pos3 == 459)
				{
					pos1 = 709; pos2 = 339; pos3 = 367;
				}
				else if (pos2 = 339 && pos3 == 367)
				{
					pos1 = 413; pos2 = 308; pos3 = 655;
				}

				cout << "fim de ciclo"<< endl;

		}
	}

	int amostras = ms.size();

	for (int linha = 0; linha < amostras; linha++)
	{
		outfile << ms[linha] << " " << qw[linha] << " " << qx[linha] << " " << qy[linha] << " " << qz[linha] << " " << x[linha] << " " << y[linha] << " " << z[linha] << endl;
	}

	outfile.close();

	Sleep(2000);
}

