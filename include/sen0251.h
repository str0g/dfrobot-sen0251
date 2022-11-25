#ifndef SEN0251_H
#define SEN0251_H

class Sen0251 {
public:
  explicit Sen0251(unsigned dev);
  ~Sen0251();

  Sen0251(const Sen0251 &) = delete;
  Sen0251(Sen0251 &&) = delete;
  Sen0251 &operator=(const Sen0251 &) = delete;
  Sen0251 &operator=(Sen0251 &&) = delete;

private:
  int file;
};

#endif
