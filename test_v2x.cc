/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

#include <fstream>
#include <string>
#include "ns3/lte-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/netanim-module.h"
#include "ns3/internet-module.h"
#include "ns3/buildings-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/uniform-planar-array.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/three-gpp-spectrum-propagation-loss-model.h"
#include "ns3/three-gpp-v2v-propagation-loss-model.h"
#include "ns3/three-gpp-channel-model.h"
#include "ns3/config-store.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("V2XUrbanScenario");

void
PrintGnuplottableBuildingListToFile (std::string filename)
{
  std::ofstream outFile;
  outFile.open (filename.c_str (), std::ios_base::out | std::ios_base::trunc);
  if (!outFile.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << filename);
      return;
    }
  uint32_t index = 0;
  for (BuildingList::Iterator it = BuildingList::Begin (); it != BuildingList::End (); ++it)
    {
      ++index;
      Box box = (*it)->GetBoundaries ();
      outFile << "set object " << index << " rect from " << box.xMin << "," << box.yMin << " to "
              << box.xMax << "," << box.yMax << std::endl;
    }
}

int
main (int argc, char *argv[])
{
  LogComponentEnable ("SpectrumChannel", LOG_LEVEL_INFO);
  LogComponentEnable ("LteHelper", LOG_LEVEL_INFO);
  std::string scenario = "V2V-Urban";
  bool verbose = false;
  bool epc = true;
  Time sim_time = Seconds(200); //total simulation time
  Time time_res = MilliSeconds (10);
  double Tx_Dbm = 50.0;
  double N_Db = 10;
  double frequency = 30.0e9;
  double subcarrier_spacing = 60e3;
  double speed_tx = 60 / 3.6;
  double speed_rx = 60 / 3.6;
  uint32_t num_round = 2; //number of rounds of the vehicles
  uint32_t num_eNB = 2; //number of eNB: Do Not Change!
  uint32_t num_tod_ue = 2; //number of vehicles that really communicates: Do Not Change!
  uint32_t num_nontod_ue =
      10; //virtual vehicles just to simulate the different traffic loads(can be 10, 20, 30...)
  uint32_t nontod_interval = 500; //interval between packets of non_tod UEs: Acting as Interference
  uint32_t tod_interval = 100; //interval between packets of tod UEs

  CommandLine cmd (__FILE__);
  cmd.AddValue ("verbose", "show all log", verbose);
  cmd.AddValue ("sim_time", "simulation time", sim_time);
  cmd.AddValue ("Tx_Dbm", "Transmitting power in dBm", Tx_Dbm);
  cmd.AddValue ("N_Db", "Noise in dB", N_Db);
  cmd.AddValue ("frequency", "operating frequency in Hz", frequency);
  cmd.AddValue ("subcarrier_spacing", "subcarrier_spacing", subcarrier_spacing);
  cmd.AddValue ("verbose", "print the progress", verbose);
  cmd.AddValue ("num_eNB", "number of eNBs", num_eNB);
  cmd.AddValue ("num_round", "number of rounds", num_round);
  cmd.AddValue ("num_nontod_ue", "number of NonTod UEs", num_nontod_ue);
  cmd.AddValue ("speed_tx", "constant speed of the first vehicle less than vmax=60km/h", speed_tx);
  cmd.AddValue ("speed_rx", "constant speed of the second vehicle less than vmax=60km/h", speed_rx);
  cmd.Parse (argc, argv);

  //Creating Nodes
  NodeContainer nontod_ue_nodes;
  nontod_ue_nodes.Create (num_nontod_ue);

  NodeContainer tod_ue_nodes;
  tod_ue_nodes.Create (num_tod_ue);

  NodeContainer enb_nodes;
  enb_nodes.Create (num_eNB);

  //Create a grid of buildings
  double sizex = 200; // m
  double sizey = 125; // m
  double street_width = 20; // m
  double sizeh = 10; // m
  uint32_t num_building_x = 2;
  uint32_t num_building_y = 2;
  double maxx = (sizex + street_width) * num_building_x + street_width; //0-maxx
  double maxy = (sizey + street_width) * num_building_y + street_width; //0-maxy

  std::vector<Ptr<Building>> buildingVector;
  for (uint32_t idx = 0; idx < num_building_x; ++idx)
    {
      for (uint32_t idy = 0; idy < num_building_y; ++idy)
        {
          Ptr<Building> building;
          building = CreateObject<Building> ();

          building->SetBoundaries (Box (idx * (sizex + street_width) + street_width,
                                        idx * (sizex + street_width) + street_width + sizex,
                                        idy * (sizey + street_width) + street_width,
                                        idy * (sizey + street_width) + street_width + sizey, 0.0,
                                        sizeh));
          building->SetNRoomsX (1);
          building->SetNRoomsY (1);
          building->SetNFloors (1);
          buildingVector.push_back (building);
        }
    }
  NS_LOG_UNCOND ("building done");

  //Install Mobility Models
  //urban maximum speed_tx is 60km/h
  Ptr<MobilityModel> tod_ue_tx_mob = CreateObject<WaypointMobilityModel> ();
  Ptr<MobilityModel> tod_ue_rx_mob = CreateObject<WaypointMobilityModel> ();
  Time begin = Seconds (0.0);

  //第一个UE的轨迹：注意Vector是一个3D矢量(x,y,z)
  tod_ue_tx_mob->GetObject<WaypointMobilityModel> ()->AddWaypoint (
      Waypoint (begin, Vector (maxx / 2, street_width / 2, 1.5)));
  begin += Seconds (1.0);
  //num_round refers to the times of a vehicle (tod-UE) going around a block
  for (uint32_t i = 0; i < num_round; i++) 
    {
      tod_ue_tx_mob->GetObject<WaypointMobilityModel> ()->AddWaypoint (
          Waypoint (begin, Vector (maxx / 2, street_width / 2, 1.5)));

      begin += Seconds ((maxy - street_width) / 2 / speed_tx);

      tod_ue_tx_mob->GetObject<WaypointMobilityModel> ()->AddWaypoint (
          Waypoint (begin, Vector (maxx / 2, maxy / 2, 1.5)));

      begin += Seconds ((maxx - street_width) / 2 / speed_tx);

      tod_ue_tx_mob->GetObject<WaypointMobilityModel> ()->AddWaypoint (
          Waypoint (begin, Vector (street_width / 2, maxy / 2, 1.5)));

      begin += Seconds ((maxy - street_width) / 2 / speed_tx);

      tod_ue_tx_mob->GetObject<WaypointMobilityModel> ()->AddWaypoint (
          Waypoint (begin, Vector (street_width / 2, street_width / 2, 1.5)));

      begin += Seconds ((maxx - street_width) / 2 / speed_tx);
    }

  //第二个UE的轨迹
  begin = Seconds (0.0);
  tod_ue_rx_mob->GetObject<WaypointMobilityModel> ()->AddWaypoint (
      Waypoint (begin, Vector (maxx / 2, maxy - street_width / 2, 1.5)));
  begin += Seconds (1.0);

  for (uint32_t i = 0; i < num_round; i++)
    {

      tod_ue_rx_mob->GetObject<WaypointMobilityModel> ()->AddWaypoint (
          Waypoint (begin, Vector (maxx / 2, maxy - street_width / 2, 1.5)));

      begin += Seconds ((maxy - street_width) / 2 / speed_rx);

      tod_ue_rx_mob->GetObject<WaypointMobilityModel> ()->AddWaypoint (
          Waypoint (begin, Vector (maxx / 2, maxy / 2, 1.5)));

      begin += Seconds ((maxx - street_width) / 2 / speed_rx);

      tod_ue_rx_mob->GetObject<WaypointMobilityModel> ()->AddWaypoint (
          Waypoint (begin, Vector (maxx - street_width / 2, maxy / 2, 1.5)));

      begin += Seconds ((maxy - street_width) / 2 / speed_rx);

      tod_ue_rx_mob->GetObject<WaypointMobilityModel> ()->AddWaypoint (
          Waypoint (begin, Vector (maxx - street_width / 2, maxy - street_width / 2, 1.5)));

      begin += Seconds ((maxx - street_width) / 2 / speed_rx);
    }

  sim_time = begin;
  NS_LOG_UNCOND (sim_time);

  tod_ue_nodes.Get (0)->AggregateObject (tod_ue_tx_mob);
  tod_ue_nodes.Get (1)->AggregateObject (tod_ue_rx_mob);

  MobilityHelper mobility;
  //其他UE: 干扰性的随机走动
  mobility.SetPositionAllocator (
      "ns3::GridPositionAllocator", "MinX", DoubleValue (street_width / 2), "MinY",
      DoubleValue (street_width / 2), "DeltaX", DoubleValue (100), "DeltaY", DoubleValue (200),
      "GridWidth", UintegerValue (5), "LayoutType", StringValue ("RowFirst"));

  mobility.SetMobilityModel (
      "ns3::RandomWalk2dMobilityModel", "Bounds",
      RectangleValue (Rectangle (0, maxx - street_width, 0, maxy - street_width)), "Speed",
      StringValue ("ns3::UniformRandomVariable[Min=6.0|Max=10.0]"));

  mobility.Install (nontod_ue_nodes);

  //eNB: 固定位置
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (enb_nodes);
  AnimationInterface::SetConstantPosition (enb_nodes.Get (0), street_width / 2,
                                           maxy - street_width / 2, 15);
  AnimationInterface::SetConstantPosition (enb_nodes.Get (1), maxx - street_width / 2,
                                           street_width / 2, 15);

  //Installing Building Information
  BuildingsHelper::Install (nontod_ue_nodes);
  BuildingsHelper::Install (tod_ue_nodes);
  BuildingsHelper::Install (enb_nodes);

  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();

  //Setting epc and pgw
  if (epc)
    {
      NS_LOG_UNCOND ("enabling EPC");
    }
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);
  epcHelper->Initialize ();

  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  //Create the Internet
  PointToPointHelper p2p;
  NetDeviceContainer internet_devs;
  p2p.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (10)));
  p2p.SetDeviceAttribute ("Mtu", UintegerValue (1500)); //Maximum Transmission Unit
  internet_devs = p2p.Install (pgw, remoteHost);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internet_ifaces = ipv4.Assign (internet_devs);
  Ipv4Address remoteHost_addr = internet_ifaces.GetAddress (1);

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  //Install Lte Devices
  NetDeviceContainer enb_devs;
  NetDeviceContainer nontod_ue_devs;
  NetDeviceContainer tod_ue_devs;
  enb_devs = lteHelper->InstallEnbDevice (enb_nodes);
  nontod_ue_devs = lteHelper->InstallUeDevice (nontod_ue_nodes);
  tod_ue_devs = lteHelper->InstallUeDevice (tod_ue_nodes);

  // for internetworking purposes, consider together Non-Tod UEs and Tod UEs
  NodeContainer ue_nodes;
  NetDeviceContainer ue_devs;
  ue_nodes.Add (nontod_ue_nodes);
  ue_nodes.Add (tod_ue_nodes);
  ue_devs.Add (nontod_ue_devs);
  ue_devs.Add (tod_ue_devs);
  //Install Ip Stack on the UEs
  internet.Install (ue_nodes);
  Ipv4InterfaceContainer ue_ifaces = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ue_devs));

  //Attach the UEs to the eNBs (needs to be done after IP address configuration)
  lteHelper->Attach (tod_ue_devs);
  lteHelper->Attach (nontod_ue_devs);

  //Channel Condition Settings
  Ptr<ChannelConditionModel> m_condmodel = CreateObject<ThreeGppV2vUrbanChannelConditionModel> ();
  m_condmodel->SetAttribute ("UpdatePeriod", TimeValue (MilliSeconds (100)));

  Ptr<ThreeGppChannelModel> channel_model = CreateObject<ThreeGppChannelModel> ();
  channel_model->SetAttribute ("Scenario", StringValue (scenario));
  channel_model->SetAttribute ("Frequency", DoubleValue (frequency));
  channel_model->SetAttribute ("ChannelConditionModel", PointerValue (m_condmodel));
  channel_model->SetAttribute ("vScatt", DoubleValue (60 / 3.6));

  //PathLoss Settings 大尺度衰落
  lteHelper->SetAttribute ("PathlossModel",
                           StringValue ("ns3::ThreeGppV2vUrbanPropagationLossModel"));
  lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (frequency));
  lteHelper->SetPathlossModelAttribute ("ShadowingEnabled", BooleanValue (false));
  lteHelper->SetPathlossModelAttribute ("ChannelConditionModel", PointerValue (m_condmodel));

  //Spectrum Fading Settings 小尺度衰落
  lteHelper->SetAttribute ("FadingModel",
                           StringValue ("ns3::ThreeGppSpectrumPropagationLossModel"));
  lteHelper->SetFadingModelAttribute ("ChannelModel", PointerValue (channel_model));

  //Here we are using UDP Protocol as the default mode
  NS_LOG_UNCOND ("setting up applications");
  uint32_t dlport = 10000;
  uint32_t ulport = 20000;
  // randomize a bit start times to avoid simulation artifacts
  // (e.g., buffer overflows due to packet transmissions happening
  // exactly at the same time)
  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  startTimeSeconds->SetAttribute ("Min", DoubleValue (0));
  startTimeSeconds->SetAttribute ("Max", DoubleValue (nontod_interval / 1000));

  for (uint32_t i = 0; i < ue_nodes.GetN (); i++)
    {
      Ptr<Node> ue_node = ue_nodes.Get (i);
      Ptr<Ipv4StaticRouting> ue_static_routing =
          ipv4RoutingHelper.GetStaticRouting (ue_node->GetObject<Ipv4> ());
      ue_static_routing->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  //Installing and Starting the Applications on the Non-TOD-UEs and TOD-UEs and the Remotehost
  for (uint32_t i = 0; i < nontod_ue_nodes.GetN (); i++)
    {
      ulport++;
      dlport++;
      ApplicationContainer server_apps;
      ApplicationContainer client_apps;

      PacketSinkHelper dl_packetsinkHelper ("ns3::UdpSocketFactory",
                                            InetSocketAddress (Ipv4Address::GetAny (), dlport));
      PacketSinkHelper ul_packetsinkHelper ("ns3::UdpSocketFactory",
                                            InetSocketAddress (Ipv4Address::GetAny (), ulport));

      //Installling DL application for Non-TOD-UEs
      NS_LOG_UNCOND ("installing UDP DL app for Non-tod UE " << i);
      UdpClientHelper dl_client (ue_ifaces.GetAddress (i), dlport);
      client_apps.Add (dl_client.Install (remoteHost));
      dl_client.SetAttribute ("Interval", TimeValue (MilliSeconds (nontod_interval)));
      dl_client.SetAttribute ("MaxPackets", UintegerValue (100000000));
      server_apps.Add (dl_packetsinkHelper.Install (ue_nodes.Get (i)));

      //Installing UL application for Remotehost
      NS_LOG_UNCOND ("installing UDP UL app for Non-tod UE " << i);
      UdpClientHelper ul_client (remoteHost_addr, ulport);
      client_apps.Add (ul_client.Install (ue_nodes.Get (i)));
      ul_client.SetAttribute ("Interval", TimeValue (MilliSeconds (nontod_interval)));
      ul_client.SetAttribute ("MaxPackets", UintegerValue (100000000));
      server_apps.Add (ul_packetsinkHelper.Install (remoteHost));

      //Enabling DL EpcTft
      Ptr<EpcTft> tft = Create<EpcTft> ();
      EpcTft::PacketFilter dlpf;
      dlpf.localPortStart = dlport;
      dlpf.localPortEnd = dlport;
      tft->Add (dlpf);

      //Enabling UL EpcTft
      EpcTft::PacketFilter ulpf;
      ulpf.remotePortStart = ulport;
      ulpf.remotePortEnd = ulport;
      tft->Add (ulpf);

      //Enabling EPS Bearer
      EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
      lteHelper->ActivateDedicatedEpsBearer (ue_devs.Get (i), bearer, tft);

      server_apps.Start (Seconds (startTimeSeconds->GetValue ()));
      client_apps.Start (Seconds (startTimeSeconds->GetValue ()));
    }

  for (uint32_t i = nontod_ue_nodes.GetN (); i < ue_nodes.GetN (); i++)
    {
      ulport++;
      dlport++;
      ApplicationContainer server_apps;
      ApplicationContainer client_apps;

      PacketSinkHelper dl_packetsinkHelper ("ns3::UdpSocketFactory",
                                            InetSocketAddress (Ipv4Address::GetAny (), dlport));
      PacketSinkHelper ul_packetsinkHelper ("ns3::UdpSocketFactory",
                                            InetSocketAddress (Ipv4Address::GetAny (), ulport));

      //Installling DL application for Tod-UEs
      NS_LOG_UNCOND ("installing UDP DL app for tod UE " << i);
      UdpClientHelper dl_client (ue_ifaces.GetAddress (i), dlport);
      client_apps.Add (dl_client.Install (remoteHost));
      dl_client.SetAttribute ("Interval", TimeValue (MilliSeconds (tod_interval)));
      dl_client.SetAttribute ("MaxPackets", UintegerValue (100000000));
      server_apps.Add (dl_packetsinkHelper.Install (ue_nodes.Get (i)));

      //Installing UL application for Remotehost
      NS_LOG_UNCOND ("installing UDP UL app for tod UE " << i);
      UdpClientHelper ul_client (remoteHost_addr, ulport);
      client_apps.Add (ul_client.Install (ue_nodes.Get (i)));
      ul_client.SetAttribute ("Interval", TimeValue (MilliSeconds (tod_interval)));
      ul_client.SetAttribute ("MaxPackets", UintegerValue (100000000));
      server_apps.Add (ul_packetsinkHelper.Install (remoteHost));

      //Enabling DL EpcTft
      Ptr<EpcTft> tft = Create<EpcTft> ();
      EpcTft::PacketFilter dlpf;
      dlpf.localPortStart = dlport;
      dlpf.localPortEnd = dlport;
      tft->Add (dlpf);

      //Enabling UL EpcTft
      EpcTft::PacketFilter ulpf;
      ulpf.remotePortStart = ulport;
      ulpf.remotePortEnd = ulport;
      tft->Add (ulpf);

      //Enabling EPS Bearer
      EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
      lteHelper->ActivateDedicatedEpsBearer (ue_devs.Get (i), bearer, tft);

      server_apps.Start (Seconds (startTimeSeconds->GetValue ()));
      client_apps.Start (Seconds (startTimeSeconds->GetValue ()));
    }

  //Enabling Traces for the following layers
  lteHelper->EnableMacTraces ();
  lteHelper->EnableRlcTraces ();
  lteHelper->EnablePdcpTraces ();


  //Updating Building Information in buildings.txt
  PrintGnuplottableBuildingListToFile ("buildings.txt");
  //Netanim Settings
  AnimationInterface anim ("scratch/test_v2x.xml");
  anim.UpdateNodeDescription (tod_ue_nodes.Get (0), "UE0"); //Ptr<node> 和description字符串
  anim.UpdateNodeDescription (tod_ue_nodes.Get (1), "UE1");
  anim.UpdateNodeDescription (enb_nodes.Get (0), "eNB0");
  anim.UpdateNodeDescription (enb_nodes.Get (1), "eNB1");
  anim.SetMaxPktsPerTraceFile (5000000000);

  Simulator::Stop (sim_time);
  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}