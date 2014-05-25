#include <string>
#include "BrianWrapper.h"
#include <iostream>
#include <fstream>
int main(int argc, char ** argv)
{
	/* leggo un file con i dati in input.
	 * Il formato e':
	 * nome valore
	 */
	std::string inputFile;
	if(argc >= 2)
		inputFile = std::string(argv[1]);
	else
		inputFile = "BrianTest.txt";
	
	std::ifstream infile(inputFile.c_str());
	if(!infile.is_open())
	{
		std::cout << "error opening file" << std::endl;
		return 1;
	}
	
	/* creo il contenitore per i dati di brian e lo riempio */
	BrianWrapper::DataContainer input, output;
	std::string name;
	float value;
	
	while(1)
	{
			infile >> name >> value;
			if(infile.eof())
				break;
			input[name] = value;
	}

	/* creo l'oggetto brian */
	BrianWrapper brian(BRIAN_CONFIG_PATH,2);
	
	/* esegui una iterazione di brian */
	output = brian.execute(input);
}
