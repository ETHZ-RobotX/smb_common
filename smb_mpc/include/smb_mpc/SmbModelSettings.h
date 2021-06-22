//
// Created by johannes on 29.04.19.
//

#pragma once

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <iostream>
#include <string>

namespace smb_path_following {

class SmbModelSettings {
 public:
  SmbModelSettings() : recompileLibraries_(false) {}

  virtual ~SmbModelSettings() = default;

  /** flag to generate dynamic files **/
  bool recompileLibraries_;
  std::string systemName_;

  virtual void loadSettings(const std::string& filename, bool verbose = true);
};

inline void SmbModelSettings::loadSettings(const std::string& filename, bool verbose /*= true*/) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  if (verbose) std::cerr << "\n #### Robot Model Settings:" << std::endl;
  if (verbose) std::cerr << " #### ==================================================" << std::endl;

  try {
    recompileLibraries_ = pt.get<bool>("model_settings.recompileLibraries");
    systemName_ = pt.get<std::string>("model_settings.systemName");
    if (verbose) std::cerr << " #### recompileLibraries ..... " << recompileLibraries_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### recompileLibraries ..... " << recompileLibraries_ << "\t(default)" << std::endl;
  }

  if (verbose) std::cerr << " #### ================================================ ####" << std::endl;
}

}  // namespace smb_path_following
