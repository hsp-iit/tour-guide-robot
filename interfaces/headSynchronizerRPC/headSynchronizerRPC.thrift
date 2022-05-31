service headSynchronizerRPC {
    bool say(1:string text);
    bool pauseSpeaking();
    bool continueSpeaking();
    bool reset();
    bool isSpeaking();
    bool isHearing();
    bool startHearing();
    bool stopHearing();
    bool sadFaceWarning();
    bool sadFaceError();
    bool busyFaceError();
    bool happyFace();
    bool busyFace();
}