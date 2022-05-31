service tourManagerRPC {
    bool sendError(1:string error);
    bool recovered();
    bool sendToPoI();
    bool isAtPoI();
    string getCurrentPoIName();
}
