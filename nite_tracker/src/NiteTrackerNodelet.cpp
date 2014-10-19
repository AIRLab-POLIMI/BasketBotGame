#include "NiteTracker.h"
#include <nodelet/nodelet.h>

namespace nite_tracker
{
	class NiteTrackerNodelet : public nodelet::Nodelet
	{
	public:
		void onInit() {
			lp.reset(new NiteTracker(getNodeHandle()));
		};
	
		boost::shared_ptr<NiteTracker> lp;
	};
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(nite_tracker, NiteTrackerNodelet, nite_tracker::NiteTrackerNodelet, nodelet::Nodelet);
