#include <string>
#include <map>
class MrBrian;

class SpecialFloat //it defaults to 0
{
	float value;
	public:
	SpecialFloat() :value(0){};
	SpecialFloat(float a) :value(a){};
	operator float() const {return value;}
	
	SpecialFloat & operator=(SpecialFloat a) {value = a.value;return *this;}
	///SpecialFloat & operator=(float a) {value = a;return *this;}
	
};

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
	void setVerbosity(unsigned int);
	typedef std::map<std::string,SpecialFloat> DataContainer;
	DataContainer execute(const DataContainer);
};