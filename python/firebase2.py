import firebase_admin
import google.cloud
from firebase_admin import credentials, firestore


cred = credentials.Certificate("./ServiceAccountKey.json")
firebase_admin.initialize_app(cred)

db = firestore.client()
doc_ref = db.collection(u'Slots').document(u'slot-1')
data = "slot-3"
doc_ref = db.collection(u'Slots').document(data)
doc_ref.update({u'Status': 'True'})
status = True
doc_ref.update({u'Status': status})
# try:
#     doc = doc_ref.get()
#     print(u'Document data: {}'.format(doc.to_dict()))
# except google.cloud.exceptions.NotFound:
#     print(u'No such document!')