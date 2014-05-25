#include <string>
#include <map>
class MrBrian;
class BrianWrapper
{
private:
	static const unsigned int MAX_FILENAME_SIZE;
	static const std::string FILES_LIST;
	unsigned int verbosity;
	MrBrian* brian;
	public:
	BrianWrapper(std::string configDir,unsigned int verbosity = 0);
	~BrianWrapper();
	
	typedef std::map<std::string,float> DataContainer;
	DataContainer execute(const DataContainer);
};