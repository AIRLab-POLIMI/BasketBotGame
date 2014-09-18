#include <string>
#include "BrianWrapper.h"
#include <iostream>
#include <fstream>
#include <ros/package.h>

int main(int argc, char ** argv)
{
	/* leggo un file con i dati in input.
	 * Il formato e':
	 * nome valore
	 */
	
	if(argc <2)
	{
		std::cout << "Usage: DataList.txt";
		return 0;
	}
	std::ifstream infile(argv[1]);
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
//std::string brian_config_path = ".";
std::string brian_config_path = ros::package::getPath("basketbot_brain") + "/config";
//std::cout << brian_config_path <<std::endl;
	/* creo l'oggetto brian */
	BrianWrapper brian(brian_config_path,2);
	
	/* esegui una iterazione di brian */
	output = brian.execute(input);
}
