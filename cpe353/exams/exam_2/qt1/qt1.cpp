//
// First qt Hello World example
// Christopher Bero
//

#include <QApplication>
#include <QLabel>

int main (int argc, char* argv[])
{
	QApplication myApp(argc, argv);
	QLabel label("Hello World");
	label.show();
	return myApp.exec();
}

