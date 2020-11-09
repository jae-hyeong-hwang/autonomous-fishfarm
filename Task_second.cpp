/*Author : Jaehyeong Hwang
second trial : 09.11.2020
still writing*/

/*

Send PlanControl command to start a plan consisting of a single FollowReference maneuver
Guide the vehicle by sending Reference messages and listening to FollowRefState and Abort messages
If Abort is received, stop controlling the vehicle immediately

*/


#include <DUNE/DUNE.hpp>

using DUNE_NAMESPACES;

namespace Autofish
{
namespace farm
{
struct Arguments
{
float vertical_tolerance;
float horizontal_tolerance;
float loiter_radius;
//waypoints
float default_speed;
float default_speed_units;
float vehicle_type;
float default_z;
float pc_id;
float plan_id_;

float flags;
float lat;
float lon;
float radius;

float s = 25;
float h = 10;
float W = 100;
};

struct Task: public DUNE::Tasks::Task
{
//static const float ServiceModeTime = 10.0;
float m_last_depth; // last caravela depth

bool m_moving;
bool m_got_reference;
double m_last_ref_time;

IMC::FollowReference m_follow_ref;
IMC::EstimatedState m_estate;
IMC::FollowRefState m_ref_state;
IMC::PlanControlState m_plan_control_state;
IMC::DesiredPath m_desired_path;

Arguments m_args;

Task(const std::string& name, Tasks::Context& ctx):
DUNE::Tasks::Task(name, ctx)
{

param("Loitering Radius", m_args.loiter_radius)
.defaultValue("5")
.units(Units::Meter)
.description("Radius of loitering circle after arriving at destination");

param("Vehicle Type", m_args.vehicle_type)
.defaultValue("AUV")
.description("Vehicle type (AUV or UAV), default AUV");

param("Horizontal Tolerance", m_args.horizontal_tolerance)
.defaultValue("15.0").units(Units::Meter)
.description("Minimum distance required to consider that the vehicle has arrived at the reference (XY)");

param("Default Speed", m_args.default_speed)
.defaultValue("50")
.description("Speed to use in case no speed is given by reference source.");

param("Default Speed Units", m_args.default_speed_units)
.defaultValue("%")
.description("Units to use for default speed (one of 'm/s', 'rpm' or '%').");

param("Default Z", m_args.default_z)
.defaultValue("0")
.units(Units::Meter)
.description("Default z when no vertical reference is given.");

param("PlanControl ID", m_args.pc_id)
.defaultValue("10000")
.minimumValue("0");

//Register Callbacks
bind<IMC::FollowReference>(this);
bind<IMC::EstimatedState>(this);
bind<IMC::Abort>(this);
}


//! Initialize resources.
void
onResourceInitialization() override
{
requestDeactivation();
}

void
startPlan(void)
{
IMC::PlanControl pc;
pc.plan_id = plan_id_;
pc.op = IMC::PlanControl::PC_START; //operation
pc.type = IMC::PlanControl::PC_REQUEST; //type
pc.flags = IMC::PlanControl::FLG_IGNORE_ERRORS;
pc.request_id = 1000;

IMC::FollowReference man;
man.control_src = 0XFFFF;
man.control_ent = 255;
man.loiter_radius = 25.0;
man.timeout = 0.0;

IMC::PlanManeuver pm;
pm.maneuver_id = "1";
pm.data.set(man);

IMC::PlanSpecification ps;
ps.plan_id = pc.plan_id;
ps.start_man_id = pm.maneuver_id;
ps.maneuver.push_back(pm);
pc.arg.set(ps);
//arg request/reply argument
//start_man_id starts a maneuver

dispatch(pc);
}

void
consume(const IMC::FollowReference* msg)
{
// do I have to include these 4 lines?
m_moving = false;
m_got_reference = false;
m_follow_ref = *msg;
m_last_ref_time = Clock::get();

m_follow_ref.flags = Reference::FLAG_LOCATION;
m_follow_ref.lat = m_estate.lat;
m_follow_ref.lon = m_estate.lon;
m_follow_ref.radius = m_args.loiter_radius;

// Notify maneuver was activated
m_ref_state.reference.set(m_follow_ref);
m_ref_state.proximity = IMC::FollowRefState::PROX_FAR;
m_ref_state.state = IMC::FollowRefState::FR_WAIT;
m_ref_state.control_ent = msg -> control_ent;
m_ref_state.control_src = msg -> control_src;
dispatch(m_ref_state);
}

void
consume(const IMC::EstimatedState* msg)
{
if (msg->getSource() != getSystemId())
return;

m_estate = *msg;
checkTimeout();
}

void
updateCoordinates(void)
{
	m_estate.lat = m_estate.lat + (m_estate.x * 2 * pi())/40075000;
	m_estate.lon = m_estate.lon + (m_estate.y * 2 * pi())/(40075000 * cos(m_estate.lat);
}


void lanmowerPattern(void)
{
	int lon_wp = [h h m_estate.lon m_estate.lon h h];
	int lat_wp = [m_estate.lat m_estate.lat+s m_estate.lat+s m_estate+2*s m_estate+2*s m_estate+3*s];


for (int i = 1 ; i <= 6 ; i++)
{
	m_follow_ref.lat = lat_wp[i];
	m_follow_ref.lon = lon_wp[i];

	if (m_state_lon == lon_wp[i] && m_state_lat == lat_wp[i])
	{return;}
	else
	{
		err("The vehicle didn't reach to the waypoint yet, wait for 10 seconds...")
		waitForMessages(10.0);
	}
}

dispatch(m_follow_ref);
}

void searchPattern(void)
{
	updateCoordinates();

	updateEndLoc();
	updateSpeed();
	updateEndZ();
	updateRadius();

	lanmowerPattern();
}

void consume(const IMC::Abort* msg)
{

if(msg->getDestination() != getSystemId())
return;

if(isActive())
{
war("Abort detected. Stop controlling...");
requestDeactivation();
}
};

/*
void
ControlState()
{
IMC::PlanControlState pcs;
if(pcs.getPlanId() == NULL)
return;

if (!frefStates.containsKey(controlState.getSourceName()))
return;
}
*/
/*
void consume(const IMC::EstimatedState* msg)
{
m_last_lon = msg->lon; // last longitude
m_last_lat = msg->lat; // last latitude
m_last_depth = msg->depth; // last depth
}
*/

/*
void
onMain(void)
{
while (!stopping())
{

waitForMessage(ServiceModeTime);

if (!isActive())
return;
(m_counter > m_args.limit) ? requestDeactivation() : inf("Received %d abort messages.", m_counter);
onDeactivation()
}

}
*/
void
        onDeactivation(void)
        {
          m_fref_state.state = IMC::FollowRefState::FR_WAIT;
          dispatch(m_fref_state);
				}


};
}

DUNE_TASK
