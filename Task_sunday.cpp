//***************************************************************************
// Copyright 2007-2020 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Tore Mo                                                          *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <vector>

namespace Maneuver
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Tore Mo
  namespace Test
  {
    using DUNE_NAMESPACES;
    using std::vector;


    struct Arguments
    {
      float loitering_radius;
      float default_speed;
      float default_z;
      float vertical_tolerance;
      float horizontal_tolerance;
      std::string default_speed_units;
      std::string default_z_units;

      float waiting_time;
      float h;
      float s;
      float current_lat;
      float current_lon;
    };


    struct Task: public DUNE::Tasks::Task
    {
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Arguments m_args;
      float start_time; //10sec

      IMC::Reference m_ref;
      IMC::FollowReference m_follow_ref;
      IMC::EstimatedState m_estate;
      IMC::FollowRefState m_ref_state;
      IMC::PlanControlState m_plan_control_state;
      IMC::DesiredPath m_d_path;

      bool m_caravela_control;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_caravela_control(false)
      {
        param("Waiting time", m_args.waiting_time)
        .defaultValue("10.0")
        .units(Units::Second)
        .description("Waiting time before starting Follow reference after boot");

        param("Longitudinal distance", m_args.h)
        .defaultValue("20.0")
        .units(Units::Meter)
        .description("Longitudinal distance vehicle has to go to the next waypoint");

        param("Latitudinal distance", m_args.s)
        .defaultValue("10.0")
        .units(Units::Meter)
        .description("Latitudinal distance vehicle has to go to the next waypoint");

        param("Horizontal Tolerance", m_args.horizontal_tolerance)
        .defaultValue("15.0")
        .units(Units::Meter)
        .description("Minimum distance required to consider that the vehicle has arrived at the reference (XY)");

        param("Vertical Tolerance", m_args.vertical_tolerance)
        .defaultValue("1.0")
        .units(Units::Meter)
        .description("Minimum distance required to consider that the vehicle has arrived at the reference (Z)");

        param("Default Speed", m_args.default_speed)
        .defaultValue("1.2")
        .description("Default Speed of Caravela");

        param("Default Speed Units", m_args.default_speed_units)
        .defaultValue("m/s")
        .values("m/s")
        .description("Default Speed Units of Caravela, meter per second");

        param("Loitering Radius", m_args.loitering_radius)
        .defaultValue("7.5")
        .units(Units::Meter)
        .description("Default Speed of Caravela");

        param("Default Z", m_args.default_z)
        .defaultValue("0")
        .units(Units::Meter)
        .description("Default z when no vertical reference is given.");

        param("Default Z Units", m_args.default_z_units)
        .defaultValue("DEPTH")
        .units(Units::Meter)
        .description("Units to use for default z reference (one of 'DEPTH', 'ALTITUDE' or 'HEIGHT')");

        bind<IMC::FollowRefState>(this);
        bind<IMC::EstimatedState>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {

      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }

      void updateSpeed(void)
      {
        m_d_path.speed = m_args.default_speed;
        m_d_path.speed_units = IMC::SUNITS_METERS_PS;
      }

      void consume(const IMC::EstimatedState* msg)
      {
        float pi = 3.14159265359;

        if (msg->getSource() != getSystemId())
        return;

        m_estate = *msg;
        m_estate.lon = msg -> lon;
        m_estate.lat = msg -> lat;
        //calculate position according to WGS84
        m_estate.lat = m_estate.lat + (m_estate.x * 2 * pi)/40075000;
	      m_estate.lon = m_estate.lon + (m_estate.y * 2 * pi)/(40075000 * cos(m_estate.lat));
      }

      void consume(const IMC::FollowRefState* msg)
      {
  /*      if(msg->state == IMC::FollowRefState::FR_WAIT)
          war("Hello");
  */      //first reference position of estimated position
        m_ref.flags = Reference::FLAG_LOCATION;
        m_ref.lon = m_estate.lon;
        m_ref.lat = m_estate.lat;
        m_ref.radius = m_args.loitering_radius;

        //m_ref.speed = m_args.default_speed;

        vector<double> lon_wp{m_args.h, m_args.h, m_estate.lon, m_estate.lon, m_args.h, m_args.h};
        vector<double> lat_wp{m_estate.lat, m_estate.lat + m_args.s, m_estate.lat + m_args.s, m_estate.lat + 2*m_args.s, m_estate.lat + 2*m_args.s, m_estate.lat + 3*m_args.s};

        for (double i = 0 ; i < 6 ; i++)
        {
          if (msg->proximity == IMC::FollowRefState::PROX_XY_NEAR)
          {
            m_ref.lon = lon_wp[i];
            m_ref.lat = lat_wp[i];
          }
        }

        dispatch(m_ref);
      }

      void
      abortMission(void)
      {
        inf("Abort Task...");
        IMC::PlanControl abortMission;
        abortMission.type = IMC::PlanControl::PC_REQUEST;
        abortMission.op = IMC::PlanControl::PC_STOP;
        abortMission.plan_id = "caravela_plan";
        dispatch(abortMission);
      }

      void
      onDeactivation(void)
      {
        m_caravela_control = m_plan_control_state.plan_id == "caravela_plan"
        && m_plan_control_state.state == IMC::PlanControlState::PCS_EXECUTING;

        if (m_caravela_control && !isActive())
        {
          abortMission();
        }
      }

      //! Main loop.
      void
      onMain(void)
      {
        DUNE::Time::Delay::waitNsec(m_args.waiting_time * 1000000000.0);
        war("Starting followref");

        IMC::PlanControl pc;
        pc.plan_id = "caravela_plan";
        pc.op = IMC::PlanControl::PC_START; //operation
        pc.type = IMC::PlanControl::PC_REQUEST; //type
        //pc.flags = IMC::PlanControl::FLG_IGNORE_ERRORS;
        pc.request_id = 0;

        IMC::FollowReference man;
        man.control_src = 0xFFFF;
        man.control_ent = 0xFF;
        man.loiter_radius = 7.5;
        man.timeout = 30.0;
        man.altitude_interval = 2.0; //

        IMC::PlanManeuver pm;
        pm.maneuver_id = "followref";
        pm.data.set(man);

        IMC::PlanSpecification ps;
        ps.plan_id = pc.plan_id;
        ps.start_man_id = pm.maneuver_id;
        ps.maneuvers.push_back(pm);
        pc.arg.set(ps);
        pc.flags = 0;
        pc.setDestination(m_ctx.resolver.id());

        dispatch(pc);

        DUNE::Time::Delay::waitNsec(1000000000.0);

        updateSpeed();

        while (!stopping())
        {
          onDeactivation();
          dispatch(m_ref);
        }


      }


    };
  }
}

DUNE_TASK
