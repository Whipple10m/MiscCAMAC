#include<iostream>
#include<iomanip>
#include<string>
#include<vector>
#include<map>
#include<cstdlib>
#include<cstring>
#include<cassert>

extern "C" {
#include<pcap.h>
}

#include <netpacket/packet.h>
#include <net/ethernet.h>
#include <netinet/in.h>

using namespace std;

template<class T> T ntoh(const u_int8_t* data)
{
  T t = *reinterpret_cast<const T*>(data);
  return t;
};

#if 0
template<> u_int16_t ntoh<u_int16_t>(const u_int8_t* data)
{
  u_int16_t s = *reinterpret_cast<const u_int16_t*>(data);
  return ntohs(s);
}

template<> u_int32_t ntoh<u_int32_t>(const u_int8_t* data)
{
  u_int32_t l = *reinterpret_cast<const u_int32_t*>(data);
  return ntohs(l);
}
#endif

template<> ether_header ntoh<ether_header>(const u_int8_t* data)
{
  ether_header ether = *reinterpret_cast<const ether_header*>(data);
  ether.ether_type = ntohs(ether.ether_type);
  return ether;
}

ostream& operator<< (ostream& s, const ether_header& x)
{
  s << resetiosflags(ios::basefield) << setfill(' ');
  for(int i=0; i<ETH_ALEN; i++)
    s << setfill('0') << setw(2) << setprecision(2) 
      << setiosflags( ios::hex ) << int(x.ether_shost[i]) 
      << ((i!=ETH_ALEN-1)?':':'-');
  for(int i=0; i<ETH_ALEN; i++)
    s << ((i==0)?'>':':')
      << setfill('0') << setw(2) << setprecision(2) 
      << setiosflags( ios::hex ) << int(x.ether_dhost[i]);
  s << resetiosflags(ios::basefield) << setfill(' ');
  s << ' ' << setw(4) << x.ether_type;
  return s;
}

struct llc_header
{
  u_int8_t dsap;
  u_int8_t ssap;
  u_int8_t llc1_control;
  u_int8_t padding;
  u_int8_t llc3_control;
  u_int8_t llc3_status;

  bool isAC() const { return ((llc3_control&0x6f) == 0x67); }
  bool isPF() const { return ((llc3_control&0x10) == 0x10); }
} __attribute__ ((__packed__));



ostream& operator<< (ostream& s, const llc_header& x)
{
  static char cccc[16][3]={"OK","?1","?2","?3","?4","?5","PE","?7",
			   "RS","UN","UE","?B","?C","?D","IP","IT"};
  static char rrrr[16][3]={"OK","?1","NR","?3","?4","?5","?6","?7",
			   "RS","UN","UE","?B","NE","?D","IP","IT"};
  s << setfill('0') << setw(2) << setprecision(2) 
    << setiosflags( ios::hex ) 
    << int(x.ssap)
    << "->"
    << setfill('0') << setw(2) << setprecision(2) 
    << setiosflags( ios::hex ) 
    << int(x.dsap)
    << resetiosflags(ios::basefield) << setfill(' ');

  if(x.isAC()) // AC0 or AC1
    {
      s << " AC" << (((x.llc3_control&0x80) == 0x80)?'1':'0')
	<< ' ' << (x.isPF()?"PF":"--") << ' ' 
	<< cccc[x.llc3_status&0x0f] << '/' 
	<< rrrr[(x.llc3_status&0xf0)>>4];
    }
  else if(x.llc3_control == 0x00)
    {
      s << " U"
	<< ' ' << (x.isPF()?"PF":"--");
    }
  return s;
}

struct ecp_header
{
  u_int16_t type;
  u_int16_t request_num;
  u_int16_t crate_num;
  u_int16_t host_ecc_id;
  u_int32_t host_proc_id;
  u_int16_t host_access_id;
  u_int16_t flags;
  u_int16_t status;
} __attribute__ ((__packed__));

#if 0
template<> ecp_header ntoh<ecp_header>(const u_int8_t* data)
{
  ecp_header ecp = *reinterpret_cast<const ecp_header*>(data);
  ecp.type = ntohs(ecp.type);
  ecp.request_num = ntohs(ecp.request_num);
  ecp.crate_num = ntohs(ecp.crate_num);
  ecp.host_ecc_id = ntohs(ecp.host_ecc_id);
  ecp.host_proc_id = ntohl(ecp.host_proc_id);
  ecp.host_access_id = ntohs(ecp.host_access_id);
  ecp.flags = ntohs(ecp.flags);
  ecp.status = ntohs(ecp.status);
  return ecp;
}
#endif

ostream& operator<< (ostream& s, const ecp_header& x)
{
  return s;  
}

class ECCPacketDecoder
{
public:
  ECCPacketDecoder() : m_got_packet(false), m_tv_first(0), m_conversations() {}
  void decodePacket(const struct pcap_pkthdr* header, const u_char* data);
  static void redirectHandler(u_char* that, const struct pcap_pkthdr* header,
			      const u_char *data);

private:
  void followConversation(const ether_header& mac, const llc_header& llc);
  bool isCommand(const ether_header& mac, const llc_header& llc);

  void decodeECPFrame(const ether_header& mac, const llc_header& llc,
		      const u_int8_t* data, int datalen);

  bool                            m_got_packet;
  double                          m_tv_first;

  vector<pair<ether_header,bool> > m_conversations;
};

void ECCPacketDecoder::
followConversation(const ether_header& mac, const llc_header& llc)
{
  ether_header e1;
  memcpy(e1.ether_dhost, mac.ether_dhost, ETH_ALEN);
  memcpy(e1.ether_shost, mac.ether_shost, ETH_ALEN);
  e1.ether_type = u_int16_t(llc.dsap)<<8 | u_int16_t(llc.ssap);

  ether_header e2;
  memcpy(e2.ether_dhost, mac.ether_shost, ETH_ALEN);
  memcpy(e2.ether_shost, mac.ether_dhost, ETH_ALEN);
  e2.ether_type = u_int16_t(llc.ssap)<<8 | u_int16_t(llc.dsap);
  
  for(vector<pair<ether_header,bool> >::iterator i = m_conversations.begin();
      i!=m_conversations.end(); i++)
    {
      if((memcmp(&e1, &i->first, sizeof(e1))==0)||
	 (memcmp(&e2, &i->first, sizeof(e2))==0))
	{
	  i->second = !i->second;
	  return;
	}
    }
  
  m_conversations.push_back(pair<ether_header,bool>(e1,true));
}

bool ECCPacketDecoder::
isCommand(const ether_header& mac, const llc_header& llc)
{
  ether_header e1;
  memcpy(e1.ether_dhost, mac.ether_dhost, ETH_ALEN);
  memcpy(e1.ether_shost, mac.ether_shost, ETH_ALEN);
  e1.ether_type = u_int16_t(llc.dsap)<<8 | u_int16_t(llc.ssap);

  ether_header e2;
  memcpy(e2.ether_dhost, mac.ether_shost, ETH_ALEN);
  memcpy(e2.ether_shost, mac.ether_dhost, ETH_ALEN);
  e2.ether_type = u_int16_t(llc.ssap)<<8 | u_int16_t(llc.dsap);

  for(vector<pair<ether_header,bool> >::iterator i = m_conversations.begin();
      i!=m_conversations.end(); i++)
    {
      if((memcmp(&e1, &i->first, sizeof(e1))==0)||
	 (memcmp(&e2, &i->first, sizeof(e2))==0))
	{
	  return i->second;
	}
    }
  assert(0);
}

void ECCPacketDecoder::redirectHandler(u_char* that, 
				       const struct pcap_pkthdr* header,
				       const u_char *data)
{
  ECCPacketDecoder* decoder = reinterpret_cast<ECCPacketDecoder*>(that);
  decoder->decodePacket(header, data);
}

void ECCPacketDecoder::decodePacket(const struct pcap_pkthdr* header,
				    const u_char* data)
{
  u_int16_t datalen;

  ether_header mac = ntoh<ether_header>(data);
  data += sizeof(mac);
  datalen = mac.ether_type;

  llc_header llc = ntoh<llc_header>(data);
  data += sizeof(llc);
  datalen -= sizeof(llc);

  if(llc.isAC())followConversation(mac,llc);

  double packet_time = double(header->ts.tv_sec+header->ts.tv_usec/1000000.0);
  if(!m_got_packet)m_tv_first = packet_time;

  cout << setiosflags(ios::fixed) << setiosflags(ios::left)
       << setw(8) << setprecision(4) << (packet_time-m_tv_first)
       << resetiosflags(ios::floatfield) << resetiosflags(ios::adjustfield)
       << ' ' << mac << ' ' << llc;

  if(llc.isAC())cout << ' ' << (isCommand(mac,llc)?'C':'R');

  if(datalen == 0)
    {
      cout << " NO LSDU" << endl;
      return;
    }

  u_int16_t frame_type = ntoh<u_int16_t>(data);
  cout << " FT" << frame_type;
  
 switch(frame_type)
    {
    case 7:
      decodeECPFrame(mac,llc,data,datalen);
      break;

    default:
      cout << endl;
      break;
    }
    
  m_got_packet = true;
}

void ECCPacketDecoder::
decodeECPFrame(const ether_header& mac, const llc_header& llc,
	       const u_int8_t* data, int datalen)
{
  ecp_header ecp = ntoh<ecp_header>(data);
  data += sizeof(ecp);
  datalen -= sizeof(ecp);

  cout << " REQ#" << ecp.request_num << " CR#" << ecp.crate_num
       << ' ' << ((ecp.flags&0x8000)?"Immediate":"Deferred");
  if(ecp.flags&0x0100)cout << " 1st";
  if(ecp.flags&0x0200)cout << " Last";
  cout << " Status:" << ecp.status << endl;

  if(datalen == 0)return;

  if((isCommand(mac,llc)) && (llc.dsap == 0x60))
    {
      static u_int32_t deffered_count  = 0;
      static u_int16_t deffered_length = 0;

      if((ecp.flags&0x0100) == 0)
	{
	  u_int32_t atomsize;
	  if(deffered_length)atomsize = sizeof(u_int32_t);
	  else atomsize = sizeof(u_int16_t);
	  assert((datalen % atomsize) == 0);

	  cout << "         ";
	  cout << "Count=" << datalen/atomsize << " DATA:";
	  for(int i=0; i<(deffered_length==1?12:18) && (deffered_count>0) && 
		(datalen>0) ; i++)
	    {
	      if(deffered_length)
		{
		  u_int32_t value = ntoh<u_int32_t>(data);
		  data += sizeof(value), datalen -= sizeof(value);
		  cout << ' ' << resetiosflags(ios::basefield)
		       << setfill('0') << setw(6) << setprecision(6) 
		       << setiosflags( ios::hex ) << value
		       << resetiosflags(ios::basefield) 
		       << setfill(' ');
		}
	      else
		{
		  u_int16_t value = ntoh<u_int16_t>(data);
		  data += sizeof(value), datalen -= sizeof(value);
		  cout << ' ' << resetiosflags(ios::basefield)
		       << setfill('0') << setw(4) << setprecision(4) 
		       << setiosflags( ios::hex ) << value
		       << resetiosflags(ios::basefield) 
		       << setfill(' ');
		}
	      deffered_count--;
	    }

	  if(deffered_count)
	    {
	      cout << " ...";

	      u_int32_t rdatalen;
	      rdatalen = deffered_count*atomsize;

	      if(rdatalen > datalen)
		{
		  assert( (ecp.flags&0x0200) == 0 );
		  deffered_count -= datalen/atomsize;
		  datalen = 0;
		}
	      else 
		{
		  data+=rdatalen, datalen -= rdatalen;
		  deffered_count = 0;
		}
	    }

	  cout << endl;
	}
      
      while(datalen)
	{
	  u_int8_t modifier = ntoh<u_int8_t>(data);
	  data += sizeof(modifier), datalen-=sizeof(modifier);
	  
	  u_int8_t command = ntoh<u_int8_t>(data);
	  data += sizeof(command), datalen-=sizeof(command);
	  
	  assert(command & 0x80);
	  command &= 0x7f;
	  
	  cout << "         ";
	  switch(command)
	    {
	    case 0x00:
	      cout << "NOOP";
	      break;
	      
	    case 0x01:
	      {
		static char CORNAME[][10] = 
		  { "NONE","MCA1","MCA2","ACA1","ACA2","UCS1","UCS2",
		    "UCW1","UCW2","ULS ","UQS1","UQS2","UQS3","MCA3",
		    "MCA4","MCA5","BBCMND","UCQ1","UCQ2" };
		
		cout << "CAMAC OP " << int(modifier) << ": ";
		u_int32_t num = ntoh<u_int32_t>(data);
		data += sizeof(num), datalen -= sizeof(num);

		u_int16_t fnal = ntoh<u_int16_t>(data);
		data += sizeof(fnal), datalen -= sizeof(fnal);
		
		u_int16_t length = (fnal>>0x00)&0x0001;
		u_int16_t a      = (fnal>>0x01)&0x000f;
		u_int16_t n      = (fnal>>0x05)&0x001f;
		u_int16_t f      = (fnal>>0x0a)&0x001f;

		cout << CORNAME[modifier] << " Count=" << num 
		     << " F=" << f << " N=" << n << " A" << a 
		     << (length==1?" 24 bit":" 16 bit");

		if((f & 0x0018)==0x0010) // its a write command
		  {
		    cout << " DATA:";
		    for(int i=0; i<(length==1?8:12) && datalen; i++)
		      {
			if(length)
			  {
			    u_int32_t value = ntoh<u_int32_t>(data);
			    data += sizeof(value), datalen -= sizeof(value);
			    cout << ' ' << resetiosflags(ios::basefield)
				 << setfill('0') << setw(6) << setprecision(6) 
				 << setiosflags( ios::hex ) << value
				 << resetiosflags(ios::basefield) 
				 << setfill(' ');
			  }
			else
			  {
			    u_int16_t value = ntoh<u_int16_t>(data);
			    data += sizeof(value), datalen -= sizeof(value);
			    cout << ' ' << resetiosflags(ios::basefield)
				 << setfill('0') << setw(4) << setprecision(4) 
				 << setiosflags( ios::hex ) << value
				 << resetiosflags(ios::basefield) 
				 << setfill(' ');
			  }
			num--;
		      }

		    if(num)
		      {
			cout << " ...";

			u_int32_t atomsize;
			if(deffered_length)atomsize = sizeof(u_int32_t);
			else atomsize = sizeof(u_int16_t);

			u_int32_t rdatalen = num*atomsize;

			if(rdatalen > datalen)
			  {
			    assert( (ecp.flags&0x0200) == 0 );
			    assert((datalen % atomsize) == 0);

			    deffered_count = num - datalen/atomsize;
			    deffered_length = length;
			    datalen = 0;
			  }
			else data+=rdatalen, datalen -= rdatalen;
		      }
		  }
		
		assert(datalen >= 0);
	      }
	      break;
	      
	    case 0x02:
	      cout << "Set MAX-NOINT, value=";
	      {
		u_int16_t value = ntoh<u_int16_t>(data);
		data += sizeof(value), datalen -= sizeof(value);
		cout << value;
	      }
	      break;

	    case 0x03:
	      cout << "Set wait timer: " << int(modifier)*10 << " msec";
	      break;

	    case 0x05:
	      cout << "UN";
	      // run on
	    case 0x04:
	      cout << "Book module: S=" << modifier;
	      break;

	    case 0x07:
	      cout << "UN";
	      // run on
	    case 0x06:
	      cout << "Book LAM: S=" << modifier;
	      break;
	    case 0x08:
	      cout << "Attach to LAM: S=" << modifier;
	      break;

	    case 9:
	      cout << "Generate Dataway Initialize";
	      break;

	    case 10:
	      cout << "Generate Crate Clear";
	      break;

	    case 11:
	      cout << ((modifier==1)?"Set":"Clear") << " dataway inhibit";
	      break;

	    case 12:
	      cout << "Test dataway inhibit";
	      break;

	    case 13:
	      cout << ((modifier==1)?"En":"Dis") << "able crate demand";
	      break;

	    case 14:
	    case 15:
	      cout << "Test crate demand " 
		   << ((command==14)?"enabled":"present");
	      break;
	      
	    case 16:
	      cout << "Set LAM access mode = " << modifier;
	      break;

	    case 17:
	    case 18:
	      cout << ((command==14)?"Clear":"Test") << " LAM S=" << modifier;
	      break;

	    case 19:
	      cout << "Inform host of LAM S=" << modifier;
	      break;
	      
	    case 20:
	      cout << "Change security table entry, ";
	      if(modifier == 0)cout << "add";
	      else if(modifier == 1)cout << "update";
	      else if(modifier == 2)cout << "delete";
	      break;
	      
	    case 21:
	      cout << "Read booking table";
	      break;

	    case 22:
	      cout << "Read timestamps";
	      break;

	    case 23:
	      cout << "Read statistics data";
	      break;

	    case 24:
	      cout << "Read trace buffer";
	      break;

	    case 42:
	      cout << "Set maximum no-Q count, value=";
	      {
		u_int16_t value = ntoh<u_int16_t>(data);
		data += sizeof(value), datalen -= sizeof(value);
		cout << value;
	      }
	      break;
	    }
	  cout << endl;
	}
    }
  else 
    {
      while(datalen)
	{
	  int16_t blocklen = ntoh<int16_t>(data);
	  data += sizeof(blocklen);
	  datalen -= sizeof(blocklen);

	  cout << "         DATA: " << setw(4) << blocklen << " words. ";
	  blocklen = abs(blocklen);
	  for(int i=0; i<20 && blocklen; i++)
	    {
	      u_int16_t word = ntoh<int16_t>(data);
	      blocklen--;
	      data += sizeof(word);
	      datalen -= sizeof(word);

	      cout << ' ' << resetiosflags(ios::basefield)
		   << setfill('0') << setw(4) << setprecision(4) 
		   << setiosflags( ios::hex ) << word
		   << resetiosflags(ios::basefield) << setfill(' ');
	    }

	  if(blocklen)
	    {
	      cout << " ...";
	      data += blocklen*sizeof(u_int16_t);
	      datalen -= blocklen*sizeof(u_int16_t);
	    }
	  
	  assert(datalen >= 0);

	  cout << endl;
	}
    }
}

int main(int argc, char** argv)
{
  string progname = *argv;
  argv++, argc--;

  if(argc == 0)
    {
      cerr << "usage: " << progname << " pcap_filename" << endl;
      exit(EXIT_FAILURE);
    }

  ECCPacketDecoder decoder;

  pcap_t* pc = pcap_open_offline(*argv, 0);
  pcap_dispatch(pc, -1, ECCPacketDecoder::redirectHandler, 
		reinterpret_cast<u_char*>(&decoder));

}
