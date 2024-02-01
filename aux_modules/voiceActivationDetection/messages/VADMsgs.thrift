service VADMsgs
{
  bool run_inference(1:bool active);
  bool set_answer(1:i32 rightAnswer)
}