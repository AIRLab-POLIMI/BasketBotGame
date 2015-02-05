#include "BrianWrapper.h"
#include "brian.h"
#include <fstream>
#include <iostream>

const unsigned int BrianWrapper::MAX_FILENAME_SIZE = 150;
const std::string BrianWrapper::FILES_LIST = "BrianConfigs.ini";

static void readConfigLine(std::ifstream & ifile, char *var)
{
	std::string temp;
	ifile >> temp;
	strcpy(var,temp.c_str());
	std::cout <<temp<<std::endl;
}
void BrianWrapper::setVerbosity(unsigned int v)
{
	verbosity = v;
}

BrianWrapper::DataContainer BrianWrapper::execute(const BrianWrapper::DataContainer input)
{
	brian->flush();
	crisp_data_list *cdl = (brian->getFuzzy())->get_crisp_data_list();

	for(BrianWrapper::DataContainer::const_iterator it = input.begin(); it != input.end(); ++it)
		cdl->add(new crisp_data(it->first.c_str(),it->second,1));

	brian->run();

	for (crisp_data_list::iterator itr = cdl->begin(); itr != cdl->end(); itr++)
		delete (*itr).second;
	cdl->clear();

	command_list *com = brian->getFuzzy()->get_command_singleton_list();

	BrianWrapper::DataContainer output;

	for (command_list::iterator it = com->begin(); it != com->end(); it++) {
		output[it->first] = it->second->get_set_point();
		delete it->second;
	}
	com->clear();
	
	if(verbosity > 0) {
		if(verbosity > 1) {
			brian->debug();
			std::ifstream log("./log/action.log");
			std::cout << log.rdbuf();
		}
		std::cout << "=================="<<std::endl;
		for(BrianWrapper::DataContainer::const_iterator it = input.begin(); it != input.end(); ++it)
			std::cout << "<<    " << it->first << ": " <<it->second <<std::endl;
		std::cout << "============"<<std::endl;
		for (BrianWrapper::DataContainer::iterator it = output.begin(); it != output.end(); it++)
			std::cout << ">>    " << it->first << ": " <<it->second <<std::endl;
		std::cout << "=================="<<std::endl;
	}

	return output;
}
BrianWrapper::BrianWrapper(std::string configDir,unsigned int verbosity):verbosity(verbosity)
{
	chdir(configDir.c_str());
	std::ifstream infile(FILES_LIST.c_str());
	typedef char filename_container[MAX_FILENAME_SIZE];
	filename_container FUZZYASSOC,FUZZYSHAPES, PRIES, PRIESACTIONS, CANDOES,BEHAVIORS, WANTERS, DEFUZZYASSOC, DEFUZZYSHAPES;
	readConfigLine(infile,FUZZYASSOC);
	readConfigLine(infile,FUZZYSHAPES);
	readConfigLine(infile,PRIES);
	readConfigLine(infile,PRIESACTIONS);
	readConfigLine(infile,CANDOES);
	readConfigLine(infile,BEHAVIORS);
	readConfigLine(infile,WANTERS);
	readConfigLine(infile,DEFUZZYASSOC);
	readConfigLine(infile,DEFUZZYSHAPES);

	brian = new MrBrian(FUZZYASSOC, FUZZYSHAPES, PRIES,
	                    PRIESACTIONS, CANDOES,BEHAVIORS,
	                    WANTERS, DEFUZZYASSOC, DEFUZZYSHAPES);

}

BrianWrapper::~BrianWrapper()
{
	delete brian;
}
