def send_status(data):

    message = f"""STATE={data['state']} ALT={data['altitude']} ROC={data['roc']}SAFE={data['safe']}"""

    print("\nAFDX MESSAGE:")
    print(message)