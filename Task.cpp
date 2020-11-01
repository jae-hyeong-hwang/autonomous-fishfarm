/*Author : Jaehyeong Hwang
  First written date : 02.11.2020 
  still writing*/


#include <DUNE/DUNE.hpp>

namespace Maneuvers
{
	namespace Maneuvers
	using DUNE_NMESPACES;

	struct Arguments
	{
		float vertical_tolerance;
		float horizontal_tolerance;
		float waypoint_a;
		float waypoint_b;
		float waypoint_c;
		float waypoint_d;

		Arguments m_args;
	};

	struct Task: public DUNE::Maneuvers::Maneuver
	{


			static const float ServiceModeTime = 10.0;

			IMC::PlanControlState m_plan_control_state;
			IMC::VehicleState m_vehicle_state;


		Task(const std::string& name, Tasks::Context& ctx:
			DUNE::Maneuvers::Maneuver(name, ctx),
			{


				param("PlanControl ID", m_args.pc_id)
				.defaultValue("10000")
				.minimumValue("0");

				bindToManeuver<Task, IMC::RowsCoverage>();
				bindToManeuver<Task, IMC::Abort>();
			}




			void
			onReportEntityState(void)
			{
				dispatch(PlanControl);
			}

			void
			onManeuverActivation(void)
			{

			}

			void
			startPlan(void)
			{
				IMC::PlanControl pc;
				pc.plan_id = plan_id_;
				pc.op = IMC::PlanControl::PC_START; //operation
				pc.type = IMC::PlanControl::PC_REQUEST; //type
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
			SearchPattern()
			{
				Neptus::RowsManeuver rowsman;
				rowsman.setSpeed(900);
				rowsman.setSpeedUnits("RPM");

			}

			void
			ControlState()
			{
				IMC::PlanControlState pcs;
				if(pcs.getPlanId() == NULL)
					return;
				  if (!frefStates.containsKey(controlState.getSourceName()))
            		return;
			}

			void consume(const IMC::RowsCoverage* maneuver)
			{

			}

			void consume(const IMC::Abort* msg)
			{
				msg->getDestination

				war("Abort received, requesting deactivation..!");
				requestDeactivation();
			}




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




	}
}

DUNE_TASK
