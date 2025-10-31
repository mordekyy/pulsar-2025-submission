//Projek calculator menggunakan c++
//by : Mikhail Muhammad Althf
// NIM : 22.11.4891

#include <iostream> 

using namespace std;

int main()
{
	float a, b, hasil;
	char ariatmatika;

	cout << "program calculator \n \n";

	cout << "masukan nilai pertama: ";
	cin >> a;
	cout << "pilih operator +,-,/,*:";
	cin >> ariatmatika;
	cout << "masukan nilai kedua: ";
	cin >> b;

	
	cout << "\nHasilperhitungan: ";
	cout << a << ariatmatika << b;

	if (ariatmatika == '+') {
		hasil = a + b;
	}
	else if (ariatmatika == '-') {
		hasil = a - b;
	}
	else if (ariatmatika == '*') {
		hasil = a * b;
	}
	else if (ariatmatika == '/') {
		hasil = a / b;
	}
	else {
		cout << "operator salah" << endl;
	}

	cout << " = " << hasil << endl;

	return 0;
}