#ifndef MYNTEYE_INTERNAL_MATCH_H_
#define MYNTEYE_INTERNAL_MATCH_H_
#pragma once

#include <vector>
#include <map>
#include <mutex>

#include "mynteyed/data/types_internal.h"
#include "mynteyed/types.h"

MYNTEYE_BEGIN_NAMESPACE

class Match {
 public:
  using img_data_t = StreamData;
  using img_datas_t = std::vector<img_data_t>;

  Match();
  ~Match();

  void OnStreamDataCallback(const ImageType &type, const img_data_t& data);

  img_datas_t GetStreamDatas(const ImageType& type);

 protected:
  void OnUpdateMatchedDatas(const ImageType& type, const StreamData& data);
  img_datas_t MatchStreamDatas(const ImageType& type);
  void InitOrder(const ImageType& type);

 private:
  std::map<ImageType, img_datas_t> stream_datas_;

  std::recursive_mutex match_mutex_;

  std::uint16_t base_frame_id_ = 0;

  /**
   * 1 -- left
   * 2 -- right
   * 3 -- depth
   * */
  int is_called_ = 0;
};

MYNTEYE_END_NAMESPACE

#endif // MYNTEYE_INTERNAL_MATCH_H_
