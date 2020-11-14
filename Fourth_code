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

namespace Maneuver
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Tore Mo
  namespace Test
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      float waiting_time;
      float delta_lat;
      float delta_lon;
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
      float start_time;
      float waiting_time; //10sec

      IMC::Reference m_ref;
      IMC::FollowReference m_follow_ref;
      IMC::EstimatedState m_estate;
      IMC::FollowRefState m_ref_state;
      IMC::PlanControlState m_plan_control_state;


      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
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

        //bind<IMC::PlanControl>(this);
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
        if(msg->state == IMC::FollowRefState::FR_WAIT)
          war("Hello");

        m_ref.lon = m_estate.lon;
        m_ref.lat = m_estate.lat;

        if (msg->proximity == IMC::FollowRefState::PROX_XY_NEAR)
          updateWP(m_ref.lon, m_ref.lat);
        else
        dispatch(m_ref);
      }

      void updateWP(float current_lon, float current_lat)
      {

        current_lon = m_estate.lon;
        current_lat = m_estate.lat;
        

        int lon_wp = [m_args.h, m_args.h, current_lon, current_lon, m_args.h, m_args.h];
        int lat_wp = [current_lat, current_lat + m_args.s, current_lat + m_args.s, current_lat + 2*m_args.s, current_lat + 2*m_args.s, currrent_lat + 3*m_args.s];

        for (int i = 1 ; i <= 6 ; i++)
        {
          m_estate.lon = lon_wp[i];
          m_estate.lat = lat_wp[i];
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
        man.control_ent = 255;
        man.loiter_radius = 25.0;
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

        while (!stopping())
        {
          dispatch(m_ref);
          waitForMessages(5.0); // wait for 5 seconds
        }

      }


    };
  }
}

DUNE_TASK
